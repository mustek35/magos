import sys
import json
import requests
from datetime import datetime, timedelta
import math
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

try:
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, 
                                 QHBoxLayout, QWidget, QLineEdit, QPushButton, 
                                 QLabel, QTextEdit, QTableWidget, QTableWidgetItem,
                                 QTabWidget, QGroupBox, QFormLayout, QSpinBox,
                                 QDoubleSpinBox, QCheckBox, QComboBox, QProgressBar,
                                 QSplitter, QMessageBox, QListWidget, QFrame)
    from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt, QUrl
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    from PyQt6.QtGui import QFont, QPalette, QColor
    PYQT_VERSION = 6
except ImportError:
    print("PyQt6 no está instalado. Intentando con PyQt5...")
    try:
        from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, 
                                     QHBoxLayout, QWidget, QLineEdit, QPushButton, 
                                     QLabel, QTextEdit, QTableWidget, QTableWidgetItem,
                                     QTabWidget, QGroupBox, QFormLayout, QSpinBox,
                                     QDoubleSpinBox, QCheckBox, QComboBox, QProgressBar,
                                     QSplitter, QMessageBox, QListWidget, QFrame)
        from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt, QUrl
        from PyQt5.QtWebEngineWidgets import QWebEngineView
        from PyQt5.QtGui import QFont, QPalette, QColor
        PYQT_VERSION = 5
    except ImportError:
        print("Error: Ni PyQt6 ni PyQt5 están instalados.")
        print("Instala uno de ellos usando:")
        print("pip install PyQt6 PyQt6-WebEngine requests")
        print("o")
        print("pip install PyQt5 PyQt5-tools requests")
        sys.exit(1)

@dataclass
class Detection:
    """Clase para representar una detección del radar"""
    id: str
    timestamp: datetime
    x: float
    y: float
    speed: float
    heading: float
    confidence: float
    track_id: Optional[str] = None
    is_false_positive: bool = False

@dataclass
class Track:
    """Clase para representar una trayectoria"""
    track_id: str
    detections: List[Detection]
    start_time: datetime
    last_update: datetime
    speed: float
    heading: float
    distance_traveled: float
    is_valid: bool = True

class RadarAPI:
    """Clase para manejar la comunicación con el radar MAGOS"""
    
    def __init__(self, base_url: str, username: str, password: str):
        self.base_url = base_url
        self.username = username
        self.password = password
        self.token = None
        self.session = requests.Session()
        
    def authenticate(self) -> bool:
        """Autentica con el radar y obtiene el token"""
        try:
            auth_url = f"{self.base_url}/api/auth/login"
            response = self.session.post(auth_url, json={
                "username": self.username,
                "password": self.password
            })
            
            if response.status_code == 200:
                data = response.json()
                self.token = data.get('token')
                self.session.headers.update({'Authorization': f'Bearer {self.token}'})
                return True
            return False
        except Exception as e:
            print(f"Error de autenticación: {e}")
            return False
    
    def get_detections(self, start_time: datetime = None, end_time: datetime = None) -> List[Detection]:
        """Obtiene las detecciones del radar"""
        try:
            if not start_time:
                start_time = datetime.now() - timedelta(minutes=5)
            if not end_time:
                end_time = datetime.now()
            
            # URL basada en la estructura de la API Digifort
            url = f"{self.base_url}/Interface/Analytics/Search"
            params = {
                'StartDate': start_time.strftime('%Y.%m.%d'),
                'StartTime': start_time.strftime('%H.%M.%S.000'),
                'EndDate': end_time.strftime('%Y.%m.%d'),
                'EndTime': end_time.strftime('%H.%M.%S.000'),
                'EventTypes': 'DETECTION,PRESENCE,VEHICLE_DETECTION',
                'ResponseFormat': 'JSON',
                'AuthUser': self.username,
                'AuthPass': self.password
            }
            
            response = self.session.get(url, params=params)
            
            if response.status_code == 200:
                data = response.json()
                detections = []
                
                # Procesar las detecciones según la estructura de la API
                if 'Response' in data and 'Data' in data['Response']:
                    records = data['Response']['Data'].get('Records', [])
                    
                    for record in records:
                        detection = Detection(
                            id=str(record.get('RECORDCODE', '')),
                            timestamp=datetime.strptime(record.get('STARTDATE', ''), '%Y-%m-%d %H:%M:%S.%f'),
                            x=float(record.get('X_COORDINATE', 0)),
                            y=float(record.get('Y_COORDINATE', 0)),
                            speed=float(record.get('SPEED', 0)),
                            heading=float(record.get('HEADING', 0)),
                            confidence=float(record.get('CONFIDENCE', 0)),
                            track_id=record.get('TRACK_ID')
                        )
                        detections.append(detection)
                
                return detections
            return []
        except Exception as e:
            print(f"Error obteniendo detecciones: {e}")
            return []

class FalsePositiveFilter:
    """Filtro para eliminar falsos positivos"""
    
    def __init__(self):
        self.min_track_duration = 10.0  # segundos
        self.min_distance_traveled = 50.0  # metros
        self.min_speed_threshold = 1.0  # m/s
        self.max_stationary_time = 5.0  # segundos
        
    def filter_detections(self, detections: List[Detection]) -> List[Detection]:
        """Filtra falsos positivos basado en trayectoria y velocidad"""
        # Agrupar detecciones por track_id
        tracks = {}
        for detection in detections:
            track_id = detection.track_id or f"single_{detection.id}"
            if track_id not in tracks:
                tracks[track_id] = []
            tracks[track_id].append(detection)
        
        valid_detections = []
        
        for track_id, track_detections in tracks.items():
            track_detections.sort(key=lambda d: d.timestamp)
            
            if self._is_valid_track(track_detections):
                for detection in track_detections:
                    detection.is_false_positive = False
                    valid_detections.append(detection)
            else:
                # Marcar como falsos positivos
                for detection in track_detections:
                    detection.is_false_positive = True
                    valid_detections.append(detection)
        
        return valid_detections
    
    def _is_valid_track(self, detections: List[Detection]) -> bool:
        """Determina si una trayectoria es válida"""
        if len(detections) < 2:
            return False
        
        # Calcular duración total
        duration = (detections[-1].timestamp - detections[0].timestamp).total_seconds()
        if duration < self.min_track_duration:
            return False
        
        # Calcular distancia total recorrida
        total_distance = 0
        for i in range(1, len(detections)):
            dx = detections[i].x - detections[i-1].x
            dy = detections[i].y - detections[i-1].y
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        if total_distance < self.min_distance_traveled:
            return False
        
        # Verificar velocidad promedio
        avg_speed = total_distance / duration if duration > 0 else 0
        if avg_speed < self.min_speed_threshold:
            return False
        
        # Verificar que no esté estacionario por mucho tiempo
        stationary_time = 0
        for i in range(1, len(detections)):
            dx = detections[i].x - detections[i-1].x
            dy = detections[i].y - detections[i-1].y
            distance = math.sqrt(dx*dx + dy*dy)
            time_diff = (detections[i].timestamp - detections[i-1].timestamp).total_seconds()
            
            if distance < 5.0:  # Menos de 5 metros de movimiento
                stationary_time += time_diff
            else:
                stationary_time = 0
            
            if stationary_time > self.max_stationary_time:
                return False
        
        return True

class DataCollectionThread(QThread):
    """Hilo para recolección continua de datos"""
    
    detections_received = pyqtSignal(list)
    connection_status = pyqtSignal(bool)
    
    def __init__(self, radar_api: RadarAPI):
        super().__init__()
        self.radar_api = radar_api
        self.running = False
        self.update_interval = 5  # segundos
        
    def run(self):
        self.running = True
        last_check = datetime.now() - timedelta(minutes=1)
        
        while self.running:
            try:
                # Obtener detecciones desde la última verificación
                current_time = datetime.now()
                detections = self.radar_api.get_detections(last_check, current_time)
                
                if detections:
                    self.detections_received.emit(detections)
                
                self.connection_status.emit(True)
                last_check = current_time
                
            except Exception as e:
                print(f"Error en recolección de datos: {e}")
                self.connection_status.emit(False)
            
            self.msleep(self.update_interval * 1000)
    
    def stop(self):
        self.running = False
        self.wait()

class RadarWebView(QWebEngineView):
    """Vista web personalizada para el radar"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)

class DetectionTableWidget(QTableWidget):
    """Tabla personalizada para mostrar detecciones"""
    
    def __init__(self):
        super().__init__()
        self.setup_table()
    
    def setup_table(self):
        headers = ['ID', 'Timestamp', 'X', 'Y', 'Velocidad', 'Rumbo', 'Confianza', 'Track ID', 'Estado']
        self.setColumnCount(len(headers))
        self.setHorizontalHeaderLabels(headers)
        self.setAlternatingRowColors(True)
        
        if PYQT_VERSION == 6:
            self.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        else:
            self.setSelectionBehavior(QTableWidget.SelectRows)
    
    def add_detection(self, detection: Detection):
        row = self.rowCount()
        self.insertRow(row)
        
        items = [
            QTableWidgetItem(detection.id),
            QTableWidgetItem(detection.timestamp.strftime('%H:%M:%S')),
            QTableWidgetItem(f"{detection.x:.2f}"),
            QTableWidgetItem(f"{detection.y:.2f}"),
            QTableWidgetItem(f"{detection.speed:.2f} m/s"),
            QTableWidgetItem(f"{detection.heading:.1f}°"),
            QTableWidgetItem(f"{detection.confidence:.1f}%"),
            QTableWidgetItem(detection.track_id or "N/A"),
            QTableWidgetItem("Falso Positivo" if detection.is_false_positive else "Válida")
        ]
        
        for col, item in enumerate(items):
            if detection.is_false_positive:
                item.setBackground(QColor(255, 200, 200))  # Fondo rojizo
            self.setItem(row, col, item)
        
        # Hacer scroll al último elemento
        self.scrollToBottom()

class RadarVisualizationWidget(QWidget):
    """Widget para visualización gráfica del radar"""
    
    def __init__(self):
        super().__init__()
        self.detections = []
        self.scale = 1.0
        self.center_x = 0
        self.center_y = 0
        self.setMinimumSize(400, 400)
        
    def add_detection(self, detection: Detection):
        """Añade una detección a la visualización"""
        self.detections.append(detection)
        # Mantener solo las últimas 100 detecciones para rendimiento
        if len(self.detections) > 100:
            self.detections = self.detections[-100:]
        self.update()
    
    def paintEvent(self, event):
        """Dibuja la visualización del radar"""
        if PYQT_VERSION == 6:
            from PyQt6.QtGui import QPainter, QPen, QBrush
        else:
            from PyQt5.QtGui import QPainter, QPen, QBrush
        
        painter = QPainter(self)
        
        if PYQT_VERSION == 6:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        else:
            painter.setRenderHint(QPainter.Antialiasing)
        
        # Fondo
        painter.fillRect(self.rect(), QColor(20, 20, 40))
        
        # Centro del radar
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # Dibujar círculos concéntricos (rangos)
        painter.setPen(QPen(QColor(0, 100, 0), 1))
        for i in range(1, 6):
            radius = i * 60
            painter.drawEllipse(center_x - radius, center_y - radius, 
                              radius * 2, radius * 2)
        
        # Dibujar líneas de azimut
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            end_x = center_x + 300 * math.cos(rad)
            end_y = center_y + 300 * math.sin(rad)
            painter.drawLine(center_x, center_y, int(end_x), int(end_y))
        
        # Dibujar detecciones
        current_time = datetime.now()
        for detection in self.detections:
            # Calcular edad de la detección
            age = (current_time - detection.timestamp).total_seconds()
            if age > 60:  # No mostrar detecciones mayores a 1 minuto
                continue
            
            # Calcular posición en pantalla
            x = center_x + detection.x * 0.5
            y = center_y + detection.y * 0.5
            
            # Color basado en el estado
            if detection.is_false_positive:
                color = QColor(255, 100, 100, 150)  # Rojo semi-transparente
            else:
                color = QColor(100, 255, 100, 200)  # Verde
            
            # Tamaño basado en la edad (más reciente = más grande)
            size = max(3, 10 - int(age / 6))
            
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color, 1))
            painter.drawEllipse(int(x - size), int(y - size), size * 2, size * 2)
            
            # Dibujar vector de velocidad
            if detection.speed > 0:
                angle_rad = math.radians(detection.heading)
                vel_x = x + detection.speed * 3 * math.cos(angle_rad)
                vel_y = y + detection.speed * 3 * math.sin(angle_rad)
                
                painter.setPen(QPen(color, 2))
                painter.drawLine(int(x), int(y), int(vel_x), int(vel_y))

class MainWindow(QMainWindow):
    """Ventana principal de la aplicación"""
    
    def __init__(self):
        super().__init__()
        self.radar_api = None
        self.data_thread = None
        self.false_positive_filter = FalsePositiveFilter()
        self.all_detections = []
        self.session_start_time = None
        self.last_update_time = None
        
        self.setup_ui()
        self.setup_timer()
        
    def setup_ui(self):
        self.setWindowTitle("Radar MAGOS - Sistema de Detección Marina")
        self.setGeometry(100, 100, 1400, 900)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        main_layout = QVBoxLayout(central_widget)
        
        # Panel de conexión
        connection_group = self.create_connection_panel()
        main_layout.addWidget(connection_group)
        
        # Splitter principal
        if PYQT_VERSION == 6:
            main_splitter = QSplitter(Qt.Orientation.Horizontal)
        else:
            main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_splitter)
        
        # Panel izquierdo - Vista web del radar
        left_panel = self.create_web_panel()
        main_splitter.addWidget(left_panel)
        
        # Panel derecho - Datos y controles
        right_panel = self.create_data_panel()
        main_splitter.addWidget(right_panel)
        
        # Configurar proporciones del splitter
        main_splitter.setSizes([600, 800])
        
        # Barra de estado
        self.statusBar().showMessage("Desconectado del radar")
    
    def create_connection_panel(self) -> QGroupBox:
        """Crea el panel de conexión"""
        group = QGroupBox("Configuración de Conexión")
        layout = QFormLayout(group)
        
        # Campos de conexión
        self.url_edit = QLineEdit("http://192.168.1.100:8601")
        self.username_edit = QLineEdit("technician")
        self.password_edit = QLineEdit("magnolia")
        
        if PYQT_VERSION == 6:
            self.password_edit.setEchoMode(QLineEdit.EchoMode.Password)
        else:
            self.password_edit.setEchoMode(QLineEdit.Password)
        
        # Botones
        self.connect_btn = QPushButton("Conectar")
        self.disconnect_btn = QPushButton("Desconectar")
        self.disconnect_btn.setEnabled(False)
        
        # Status de conexión
        self.connection_status = QLabel("●")
        self.connection_status.setStyleSheet("color: red; font-size: 16px;")
        
        # Layout
        layout.addRow("URL del Radar:", self.url_edit)
        layout.addRow("Usuario:", self.username_edit)
        layout.addRow("Contraseña:", self.password_edit)
        
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.connect_btn)
        button_layout.addWidget(self.disconnect_btn)
        button_layout.addWidget(QLabel("Estado:"))
        button_layout.addWidget(self.connection_status)
        button_layout.addStretch()
        
        layout.addRow(button_layout)
        
        # Conectar señales
        self.connect_btn.clicked.connect(self.connect_to_radar)
        self.disconnect_btn.clicked.connect(self.disconnect_from_radar)
        
        return group
    
    def create_web_panel(self) -> QWidget:
        """Crea el panel de vista web"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Título
        title = QLabel("Vista Web del Radar MAGOS")
        if PYQT_VERSION == 6:
            title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        else:
            title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title)
        
        # Vista web
        try:
            self.web_view = RadarWebView()
            layout.addWidget(self.web_view)
        except Exception as e:
            # Si WebEngine no está disponible, mostrar un placeholder
            placeholder = QLabel("Vista Web no disponible\n(PyQt6-WebEngine no instalado)")
            placeholder.setStyleSheet("border: 1px solid gray; padding: 20px; text-align: center;")
            layout.addWidget(placeholder)
            self.web_view = None
        
        # Botón para recargar

        reload_btn = QPushButton("Recargar Vista Web")
        reload_btn.clicked.connect(self.reload_web_view)
        layout.addWidget(reload_btn)

        return widget

    def create_data_panel(self) -> QWidget:
        """Crea el panel de datos"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Tabs
        tab_widget = QTabWidget()
        layout.addWidget(tab_widget)

        # Tab 1: Detecciones en tiempo real
        detections_tab = self.create_detections_tab()
        tab_widget.addTab(detections_tab, "Detecciones")

        # Tab 2: Configuración de filtros
        filters_tab = self.create_filters_tab()
        tab_widget.addTab(filters_tab, "Filtros")

        # Tab 3: Estadísticas
        stats_tab = self.create_stats_tab()
        tab_widget.addTab(stats_tab, "Estadísticas")

        # Tab 4: Visualización
        viz_tab = self.create_visualization_tab()
        tab_widget.addTab(viz_tab, "Visualización")

        return widget

    def create_detections_tab(self) -> QWidget:
        """Crea la pestaña de detecciones"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Panel de control
        control_panel = QGroupBox("Control de Monitoreo")
        control_layout = QHBoxLayout(control_panel)
        
        self.auto_update_cb = QCheckBox("Actualización Automática")
        self.auto_update_cb.setChecked(True)
        
        self.update_interval_spin = QSpinBox()
        self.update_interval_spin.setRange(1, 60)
        self.update_interval_spin.setValue(5)
        self.update_interval_spin.setSuffix(" seg")
        
        self.clear_btn = QPushButton("Limpiar Tabla")
        self.clear_btn.clicked.connect(self.clear_detections)
        
        control_layout.addWidget(self.auto_update_cb)
        control_layout.addWidget(QLabel("Intervalo:"))
        control_layout.addWidget(self.update_interval_spin)
        control_layout.addWidget(self.clear_btn)
        control_layout.addStretch()
        
        layout.addWidget(control_panel)
        
        # Estadísticas rápidas
        stats_panel = QGroupBox("Estadísticas Rápidas")
        stats_layout = QFormLayout(stats_panel)
        
        self.total_detections_label = QLabel("0")
        self.valid_detections_label = QLabel("0")
        self.false_positives_label = QLabel("0")
        self.last_update_label = QLabel("Nunca")
        
        stats_layout.addRow("Total Detecciones:", self.total_detections_label)
        stats_layout.addRow("Detecciones Válidas:", self.valid_detections_label)
        stats_layout.addRow("Falsos Positivos:", self.false_positives_label)
        stats_layout.addRow("Última Actualización:", self.last_update_label)
        
        layout.addWidget(stats_panel)
        
        # Tabla de detecciones
        self.detections_table = DetectionTableWidget()
        layout.addWidget(self.detections_table)
        
        return widget
    
    def create_filters_tab(self) -> QWidget:
        """Crea la pestaña de configuración de filtros"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Configuración de filtros de falsos positivos
        filter_group = QGroupBox("Configuración de Filtros de Falsos Positivos")
        filter_layout = QFormLayout(filter_group)
        
        self.min_duration_spin = QDoubleSpinBox()
        self.min_duration_spin.setRange(1.0, 300.0)
        self.min_duration_spin.setValue(10.0)
        self.min_duration_spin.setSuffix(" seg")
        
        self.min_distance_spin = QDoubleSpinBox()
        self.min_distance_spin.setRange(1.0, 1000.0)
        self.min_distance_spin.setValue(50.0)
        self.min_distance_spin.setSuffix(" m")
        
        self.min_speed_spin = QDoubleSpinBox()
        self.min_speed_spin.setRange(0.1, 100.0)
        self.min_speed_spin.setValue(1.0)
        self.min_speed_spin.setSuffix(" m/s")
        
        self.max_stationary_spin = QDoubleSpinBox()
        self.max_stationary_spin.setRange(1.0, 60.0)
        self.max_stationary_spin.setValue(5.0)
        self.max_stationary_spin.setSuffix(" seg")
        
        filter_layout.addRow("Duración Mínima de Track:", self.min_duration_spin)
        filter_layout.addRow("Distancia Mínima Recorrida:", self.min_distance_spin)
        filter_layout.addRow("Velocidad Mínima:", self.min_speed_spin)
        filter_layout.addRow("Tiempo Máximo Estacionario:", self.max_stationary_spin)
        
        # Botón para aplicar configuración
        apply_filters_btn = QPushButton("Aplicar Configuración")
        apply_filters_btn.clicked.connect(self.apply_filter_settings)
        filter_layout.addRow(apply_filters_btn)
        
        layout.addWidget(filter_group)
        
        # Configuración de zona marítima
        zone_group = QGroupBox("Configuración de Zona Marítima")
        zone_layout = QFormLayout(zone_group)
        
        self.sea_zone_cb = QCheckBox("Activar Filtro de Zona Marítima")
        self.sea_zone_cb.setChecked(True)
        
        zone_layout.addRow(self.sea_zone_cb)
        
        layout.addWidget(zone_group)
        layout.addStretch()
        
        return widget
    
    def create_stats_tab(self) -> QWidget:
        """Crea la pestaña de estadísticas"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Estadísticas detalladas
        stats_group = QGroupBox("Estadísticas Detalladas")
        stats_layout = QFormLayout(stats_group)
        
        self.session_start_label = QLabel("No iniciada")
        self.total_session_detections = QLabel("0")
        self.avg_detections_per_minute = QLabel("0.0")
        self.filter_efficiency = QLabel("0.0%")
        
        stats_layout.addRow("Sesión Iniciada:", self.session_start_label)
        stats_layout.addRow("Total Detecciones (Sesión):", self.total_session_detections)
        stats_layout.addRow("Promedio por Minuto:", self.avg_detections_per_minute)
        stats_layout.addRow("Eficiencia del Filtro:", self.filter_efficiency)
        
        layout.addWidget(stats_group)
        
        # Log de eventos
        log_group = QGroupBox("Log de Eventos")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setReadOnly(True)
        
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        layout.addStretch()
        
        return widget
    
    def create_visualization_tab(self) -> QWidget:
        """Crea la pestaña de visualización"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Controles de visualización
        controls_group = QGroupBox("Controles de Visualización")
        controls_layout = QHBoxLayout(controls_group)
        
        self.viz_enabled_cb = QCheckBox("Habilitar Visualización")
        self.viz_enabled_cb.setChecked(True)
        
        self.show_false_positives_cb = QCheckBox("Mostrar Falsos Positivos")
        self.show_false_positives_cb.setChecked(True)
        
        self.trail_length_spin = QSpinBox()
        self.trail_length_spin.setRange(10, 200)
        self.trail_length_spin.setValue(100)
        self.trail_length_spin.setSuffix(" detecciones")
        
        controls_layout.addWidget(self.viz_enabled_cb)
        controls_layout.addWidget(self.show_false_positives_cb)
        controls_layout.addWidget(QLabel("Longitud de Rastro:"))
        controls_layout.addWidget(self.trail_length_spin)
        controls_layout.addStretch()
        
        layout.addWidget(controls_group)
        
        # Widget de visualización
        self.radar_viz = RadarVisualizationWidget()
        layout.addWidget(self.radar_viz)
        
        return widget
    
    def setup_timer(self):
        """Configura el timer para actualizaciones"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_statistics)
        self.update_timer.start(1000)  # Actualizar cada segundo
    
    def connect_to_radar(self):
        """Conecta al radar MAGOS"""
        url = self.url_edit.text()
        username = self.username_edit.text()
        password = self.password_edit.text()
        
        if not all([url, username, password]):
            QMessageBox.warning(self, "Error", "Por favor, complete todos los campos de conexión.")
            return
        
        try:
            self.radar_api = RadarAPI(url, username, password)
            
            if self.radar_api.authenticate():
                self.connection_status.setStyleSheet("color: green; font-size: 16px;")
                self.statusBar().showMessage("Conectado al radar MAGOS")
                
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                
                # Cargar vista web
                self.load_web_view()
                
                # Iniciar recolección de datos
                self.start_data_collection()
                
                # Log
                self.log_event("Conectado exitosamente al radar MAGOS")
                self.session_start_time = datetime.now()
                self.session_start_label.setText(self.session_start_time.strftime('%Y-%m-%d %H:%M:%S'))
                
            else:
                QMessageBox.critical(self, "Error de Conexión", 
                                   "No se pudo autenticar con el radar. Verifique las credenciales.")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error de conexión: {str(e)}")
    
    def disconnect_from_radar(self):
        """Desconecta del radar"""
        if self.data_thread:
            self.data_thread.stop()
            self.data_thread = None
        
        self.radar_api = None
        self.connection_status.setStyleSheet("color: red; font-size: 16px;")
        self.statusBar().showMessage("Desconectado del radar")
        
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        
        self.log_event("Desconectado del radar MAGOS")
    
    def load_web_view(self):
        """Carga la vista web del radar"""
        if self.radar_api and self.web_view:
            web_url = f"{self.radar_api.base_url}/webclient"
            self.web_view.setUrl(QUrl(web_url))
    
    def reload_web_view(self):
        """Recarga la vista web"""
        if self.web_view:
            self.web_view.reload()
    
    def start_data_collection(self):
        """Inicia la recolección de datos"""
        if self.radar_api and not self.data_thread:
            self.data_thread = DataCollectionThread(self.radar_api)
            self.data_thread.detections_received.connect(self.process_new_detections)
            self.data_thread.connection_status.connect(self.update_connection_status)
            
            # Configurar intervalo de actualización
            self.data_thread.update_interval = self.update_interval_spin.value()
            
            self.data_thread.start()
    
    def process_new_detections(self, detections: List[Detection]):
        """Procesa nuevas detecciones recibidas"""
        if not detections:
            return
        
        # Aplicar filtros de falsos positivos
        filtered_detections = self.false_positive_filter.filter_detections(detections)
        
        # Agregar a la lista total
        self.all_detections.extend(filtered_detections)
        
        # Mantener solo las últimas 1000 detecciones para rendimiento
        if len(self.all_detections) > 1000:
            self.all_detections = self.all_detections[-1000:]
        
        # Actualizar tabla si la actualización automática está habilitada
        if self.auto_update_cb.isChecked():
            for detection in filtered_detections:
                self.detections_table.add_detection(detection)
        
        # Actualizar visualización
        if hasattr(self, 'viz_enabled_cb') and self.viz_enabled_cb.isChecked():
            for detection in filtered_detections:
                if (not detection.is_false_positive or 
                    (hasattr(self, 'show_false_positives_cb') and 
                     self.show_false_positives_cb.isChecked())):
                    self.radar_viz.add_detection(detection)
        
        # Actualizar timestamp de última actualización
        self.last_update_time = datetime.now()
        self.last_update_label.setText(self.last_update_time.strftime('%H:%M:%S'))
        
        # Log
        valid_count = sum(1 for d in filtered_detections if not d.is_false_positive)
        false_positive_count = len(filtered_detections) - valid_count
        
        if filtered_detections:
            self.log_event(f"Procesadas {len(filtered_detections)} detecciones "
                          f"({valid_count} válidas, {false_positive_count} falsos positivos)")
    
    def update_connection_status(self, is_connected: bool):
        """Actualiza el estado de conexión"""
        if is_connected:
            self.connection_status.setStyleSheet("color: green; font-size: 16px;")
        else:
            self.connection_status.setStyleSheet("color: orange; font-size: 16px;")
    
    def apply_filter_settings(self):
        """Aplica la nueva configuración de filtros"""
        self.false_positive_filter.min_track_duration = self.min_duration_spin.value()
        self.false_positive_filter.min_distance_traveled = self.min_distance_spin.value()
        self.false_positive_filter.min_speed_threshold = self.min_speed_spin.value()
        self.false_positive_filter.max_stationary_time = self.max_stationary_spin.value()
        
        # Reconfigurar el hilo de datos si está activo
        if self.data_thread:
            self.data_thread.update_interval = self.update_interval_spin.value()
        
        self.log_event("Configuración de filtros actualizada")
        QMessageBox.information(self, "Configuración", "Los filtros han sido actualizados correctamente.")
    
    def clear_detections(self):
        """Limpia la tabla de detecciones"""
        self.detections_table.setRowCount(0)
        self.log_event("Tabla de detecciones limpiada")
    
    def update_statistics(self):
        """Actualiza las estadísticas en tiempo real"""
        # Estadísticas básicas
        total = len(self.all_detections)
        valid = sum(1 for d in self.all_detections if not d.is_false_positive)
        false_positives = total - valid
        
        self.total_detections_label.setText(str(total))
        self.valid_detections_label.setText(str(valid))
        self.false_positives_label.setText(str(false_positives))
        
        # Estadísticas de sesión
        self.total_session_detections.setText(str(total))
        
        # Calcular promedio por minuto
        if self.session_start_time:
            session_duration = (datetime.now() - self.session_start_time).total_seconds() / 60
            if session_duration > 0:
                avg_per_minute = total / session_duration
                self.avg_detections_per_minute.setText(f"{avg_per_minute:.1f}")
        
        # Eficiencia del filtro
        if total > 0:
            efficiency = (false_positives / total) * 100
            self.filter_efficiency.setText(f"{efficiency:.1f}%")
    
    def log_event(self, message: str):
        """Añade un evento al log"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        log_entry = f"[{timestamp}] {message}"
        self.log_text.append(log_entry)
        
        # Mantener solo las últimas 100 líneas
        if self.log_text.document().blockCount() > 100:
            cursor = self.log_text.textCursor()
            if PYQT_VERSION == 6:
                cursor.movePosition(cursor.MoveOperation.Start)
                cursor.select(cursor.SelectionType.BlockUnderCursor)
            else:
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()


class ConfigurationManager:
    """Gestor de configuración de la aplicación"""
    
    def __init__(self, config_file: str = "radar_config.json"):
        self.config_file = config_file
        self.default_config = {
            "connection": {
                "url": "http://192.168.1.100:8601",
                "username": "technician",
                "password": "magnolia"
            },
            "filters": {
                "min_track_duration": 10.0,
                "min_distance_traveled": 50.0,
                "min_speed_threshold": 1.0,
                "max_stationary_time": 5.0
            },
            "display": {
                "auto_update": True,
                "update_interval": 5,
                "show_false_positives": True,
                "trail_length": 100
            }
        }
    
    def load_config(self) -> dict:
        """Carga la configuración desde archivo"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                # Fusionar con configuración por defecto
                return self._merge_config(self.default_config, config)
        except FileNotFoundError:
            return self.default_config.copy()
        except Exception as e:
            print(f"Error cargando configuración: {e}")
            return self.default_config.copy()
    
    def save_config(self, config: dict):
        """Guarda la configuración a archivo"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"Error guardando configuración: {e}")
    
    def _merge_config(self, default: dict, user: dict) -> dict:
        """Fusiona configuración de usuario con la por defecto"""
        result = default.copy()
        for key, value in user.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._merge_config(result[key], value)
            else:
                result[key] = value
        return result


class ExportManager:
    """Gestor para exportar datos"""
    
    @staticmethod
    def export_to_csv(detections: List[Detection], filename: str):
        """Exporta detecciones a CSV"""
        import csv
        
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                # Encabezados
                headers = ['ID', 'Timestamp', 'X', 'Y', 'Speed', 'Heading', 
                          'Confidence', 'Track_ID', 'Is_False_Positive']
                writer.writerow(headers)
                
                # Datos
                for detection in detections:
                    row = [
                        detection.id,
                        detection.timestamp.isoformat(),
                        detection.x,
                        detection.y,
                        detection.speed,
                        detection.heading,
                        detection.confidence,
                        detection.track_id or '',
                        detection.is_false_positive
                    ]
                    writer.writerow(row)
                    
            return True
        except Exception as e:
            print(f"Error exportando a CSV: {e}")
            return False
    
    @staticmethod
    def export_to_json(detections: List[Detection], filename: str):
        """Exporta detecciones a JSON"""
        try:
            data = []
            for detection in detections:
                data.append({
                    'id': detection.id,
                    'timestamp': detection.timestamp.isoformat(),
                    'x': detection.x,
                    'y': detection.y,
                    'speed': detection.speed,
                    'heading': detection.heading,
                    'confidence': detection.confidence,
                    'track_id': detection.track_id,
                    'is_false_positive': detection.is_false_positive
                })
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
                
            return True
        except Exception as e:
            print(f"Error exportando a JSON: {e}")
            return False


def main():
    """Función principal"""
    app = QApplication(sys.argv)
    
    # Configurar estilo de la aplicación
    app.setStyle('Fusion')
    
    # Paleta de colores oscura
    palette = QPalette()
    if PYQT_VERSION == 6:
        palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(0, 0, 0))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
        palette.setColor(QPalette.ColorRole.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
    else:
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, QColor(0, 0, 0))
        palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
        palette.setColor(QPalette.Text, QColor(255, 255, 255))
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
        palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    
    app.setPalette(palette)
    
    # Crear y mostrar ventana principal
    window = MainWindow()
    window.show()
    
    # Ejecutar aplicación
    if PYQT_VERSION == 6:
        sys.exit(app.exec())
    else:
        sys.exit(app.exec_())


if __name__ == "__main__":
    main()

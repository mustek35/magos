import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet'
import { useEffect, useState } from 'react'

const WS_URL = 'ws://localhost:9000'

export default function App() {
  const [devices, setDevices] = useState({})

  useEffect(() => {
    const ws = new WebSocket(WS_URL)
    ws.onmessage = (evt) => {
      try {
        const data = JSON.parse(evt.data)
        if (data.id && data.lat && data.lon) {
          setDevices(prev => ({ ...prev, [data.id]: data }))
        }
      } catch {
        // ignore malformed messages
      }
    }
    return () => ws.close()
  }, [])

  const markers = Object.values(devices)

  return (
    <div style={{ display: 'flex', height: '100%' }}>
      <div style={{ flex: 1 }}>
        <MapContainer center={[-41.6659, -73.0553]} zoom={15} style={{ height: '100%', width: '100%' }}>
          <TileLayer url='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png' />
          {markers.map(m => (
            <Marker key={m.id} position={[m.lat, m.lon]}>
              <Popup>{m.id}</Popup>
            </Marker>
          ))}
        </MapContainer>
      </div>
      <div style={{ width: '200px', padding: '10px', overflowY: 'auto', background: '#f7f7f7' }}>
        <h3>Dispositivos</h3>
        <ul style={{ listStyle: 'none', padding: 0 }}>
          {markers.map(m => (
            <li key={m.id}>{m.id}</li>
          ))}
        </ul>
      </div>
    </div>
  )
}

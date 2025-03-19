import pyrplidar_c1
import matplotlib.pyplot as plt
import numpy as np
import time
import math

def main():
    lidar = pyrplidar_c1.RPlidar()
    
    # Verbindung zum RPLIDAR C1 herstellen
    # Passen Sie den Port entsprechend Ihrem System an
    # Unter Windows z.B. 'COM5', unter Linux '/dev/ttyUSB0'
    port = '/dev/ttyUSB1'  # Oder '/dev/ttyS0' bei Raspberry Pi GPIO-Anschluss
    
    print(f"Verbinde mit RPLIDAR C1 auf Port {port}...")
    if not lidar.connect(port, 460800):
        print("Fehler beim Verbinden mit dem RPLIDAR C1. Überprüfen Sie die Verbindung.")
        return
    
    print("Verbindung erfolgreich hergestellt.")
    
    # Geräteinformationen abrufen
    info = lidar.get_info()
    print("Geräteinformationen:")
    for key, value in info.items():
        print(f"  {key}: {value}")
    
    # Gesundheitsstatus abrufen
    health = lidar.get_health()
    print("Gesundheitsstatus:")
    for key, value in health.items():
        print(f"  {key}: {value}")
    
    # Scan starten
    print("Starte Scan...")
    if not lidar.start_scan():
        print("Fehler beim Starten des Scans.")
        lidar.disconnect()
        return
    
    try:
        # Matplotlib-Einstellungen für Live-Visualisierung
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='polar')
        ax.set_title('RPLIDAR C1 Scan')
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        
        line, = ax.plot([], [], 'r.', markersize=2)
        
        # Daten mehrmals abrufen für stabilere Darstellung
        while True:
            # Daten holen und aktualisieren
            scan_data = lidar.get_scan_data()
            
            if len(scan_data) > 0:
                angles = np.array([point['angle'] * math.pi / 180.0 for point in scan_data])
                distances = np.array([point['distance'] for point in scan_data])
                
                line.set_data(angles, distances)
                
                # Plot-Grenzen anpassen
                max_distance = max(distances) if distances.size > 0 else 5000
                ax.set_rmax(max_distance * 1.1)
                
                plt.pause(0.1)
        
        plt.show(block=True)
        
    except KeyboardInterrupt:
        print("Scan durch Benutzer unterbrochen.")
    finally:
        # Scan stoppen und Verbindung trennen
        print("Stoppe Scan...")
        lidar.stop()
        
        print("Trenne Verbindung...")
        lidar.disconnect()
        
        print("Programm beendet.")

if __name__ == "__main__":
    main()

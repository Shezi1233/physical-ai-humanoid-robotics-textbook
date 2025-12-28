import React from 'react';
import styles from './SimulationViewer.module.css';

interface SimulationViewerProps {
  title?: string;
  description?: string;
  simulationType?: 'gazebo' | 'unity' | 'isaac' | 'custom';
}

const SimulationViewer: React.FC<SimulationViewerProps> = ({
  title = 'Simulation Viewer',
  description = 'Interactive simulation environment',
  simulationType = 'gazebo'
}) => {
  return (
    <div className={styles.simulationViewer}>
      <div className={styles.header}>
        <h3>{title}</h3>
        <p className={styles.description}>{description}</p>
      </div>

      <div className={styles.simulationContainer}>
        <div className={styles.simulationPlaceholder}>
          <p>Simulation Environment: {simulationType.toUpperCase()}</p>
          <p>This component would typically display an embedded simulation viewer</p>
        </div>

        <div className={styles.controls}>
          <button className={styles.controlBtn}>Play</button>
          <button className={styles.controlBtn}>Pause</button>
          <button className={styles.controlBtn}>Reset</button>
        </div>
      </div>

      <div className={styles.infoPanel}>
        <h4>Simulation Information</h4>
        <ul>
          <li>Physics engine: Accurate simulation of real-world physics</li>
          <li>Real-time rendering: High-fidelity visualization</li>
          <li>Sensor simulation: LiDAR, cameras, IMU, and other sensors</li>
        </ul>
      </div>
    </div>
  );
};

export default SimulationViewer;
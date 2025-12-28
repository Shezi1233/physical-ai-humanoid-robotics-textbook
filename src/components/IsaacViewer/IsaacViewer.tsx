import React from 'react';
import styles from './IsaacViewer.module.css';

interface IsaacViewerProps {
  title?: string;
  description?: string;
  viewerType?: 'isaac-sim' | 'isaac-ros' | 'nav2' | 'custom';
}

const IsaacViewer: React.FC<IsaacViewerProps> = ({
  title = 'Isaac Viewer',
  description = 'NVIDIA Isaac ecosystem visualization',
  viewerType = 'isaac-sim'
}) => {
  return (
    <div className={styles.isaacViewer}>
      <div className={styles.header}>
        <h3>{title}</h3>
        <p className={styles.description}>{description}</p>
      </div>

      <div className={styles.viewerContainer}>
        <div className={styles.viewerPlaceholder}>
          <p>Isaac Component: {viewerType.toUpperCase()}</p>
          <p>This component would typically display Isaac Sim/ROS visualization</p>
        </div>

        <div className={styles.controls}>
          <button className={styles.controlBtn}>Play</button>
          <button className={styles.controlBtn}>Pause</button>
          <button className={styles.controlBtn}>Reset</button>
        </div>
      </div>

      <div className={styles.infoPanel}>
        <h4>Isaac Ecosystem Information</h4>
        <ul>
          <li>GPU-accelerated: Leverages NVIDIA hardware for performance</li>
          <li>Synthetic data: Photorealistic scene generation</li>
          <li>VSLAM: Visual SLAM with GPU acceleration</li>
          <li>Navigation: Nav2 integration for humanoid movement</li>
        </ul>
      </div>
    </div>
  );
};

export default IsaacViewer;
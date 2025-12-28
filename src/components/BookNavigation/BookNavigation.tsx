import React from 'react';
import styles from './BookNavigation.module.css';
import Link from '@docusaurus/Link';

interface BookNavigationProps {
  currentModule?: string;
  currentChapter?: string;
  prevLink?: string;
  nextLink?: string;
  moduleName?: string;
  chapterName?: string;
}

const BookNavigation: React.FC<BookNavigationProps> = ({
  currentModule = '',
  currentChapter = '',
  prevLink,
  nextLink,
  moduleName = 'Module',
  chapterName = 'Chapter'
}) => {
  return (
    <div className={styles.bookNavigation}>
      <div className={styles.moduleInfo}>
        <h4>{moduleName}</h4>
        <p>{chapterName}</p>
      </div>

      <div className={styles.navigationControls}>
        {prevLink ? (
          <Link to={prevLink} className={`${styles.navButton} ${styles.prev}`}>
            ← Previous
          </Link>
        ) : (
          <span className={`${styles.navButton} ${styles.prev} ${styles.disabled}`}>
            ← Previous
          </span>
        )}

        <div className={styles.chapterProgress}>
          <span className={styles.currentModule}>{currentModule}</span>
          <span className={styles.currentChapter}>{currentChapter}</span>
        </div>

        {nextLink ? (
          <Link to={nextLink} className={`${styles.navButton} ${styles.next}`}>
            Next →
          </Link>
        ) : (
          <span className={`${styles.navButton} ${styles.next} ${styles.disabled}`}>
            Next →
          </span>
        )}
      </div>

      <div className={styles.moduleNavigation}>
        <h5>Module Contents</h5>
        <ul>
          <li><Link to="/docs/module-4-vla-llm-robotics/chapter-1-voice-to-action">Chapter 1: Voice-to-Action</Link></li>
          <li><Link to="/docs/module-4-vla-llm-robotics/chapter-2-cognitive-planning">Chapter 2: Cognitive Planning</Link></li>
          <li><Link to="/docs/module-4-vla-llm-robotics/chapter-3-autonomous-humanoid">Chapter 3: Autonomous Humanoid</Link></li>
        </ul>
      </div>
    </div>
  );
};

export default BookNavigation;
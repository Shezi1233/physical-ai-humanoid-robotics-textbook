import React from 'react';
import clsx from 'clsx';
import styles from './WeekTopics.module.css';
import { getTopicsForWeekRange } from '@site/src/utils/moduleData';

interface WeekTopicsProps {
  weekRange: string;
  startWeek: number;
  endWeek: number;
}

const WeekTopics: React.FC<WeekTopicsProps> = ({ weekRange, startWeek, endWeek }) => {
  const topics = getTopicsForWeekRange(startWeek, endWeek);

  return (
    <div className={styles.weekTopicsContainer}>
      <h2 className={styles.weekHeader}>Topics for {weekRange}</h2>

      {topics.length === 0 ? (
        <div className={styles.noTopics}>
          <p>No topics found for this week range.</p>
        </div>
      ) : (
        <div className={styles.topicsGrid}>
          {topics.map((topic, index) => (
            <div key={`${topic.moduleId}-${topic.chapterId}`} className={styles.topicCard}>
              <div className={styles.topicHeader}>
                <h3 className={styles.topicTitle}>{topic.title}</h3>
                <span className={styles.moduleBadge}>{topic.moduleName}</span>
              </div>
              <div className={styles.topicContent}>
                <p className={styles.topicDescription}>{topic.description}</p>
                <div
                  className={styles.topicText}
                  dangerouslySetInnerHTML={{ __html: topic.content.replace(/\n/g, '<br />') }}
                />
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default WeekTopics;
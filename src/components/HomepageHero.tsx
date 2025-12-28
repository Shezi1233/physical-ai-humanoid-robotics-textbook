import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Heading from '@theme/Heading';
import styles from './HomepageHero.module.css';

function HomepageHero() {
  const { siteConfig } = useDocusaurusContext();

  // Weekly topics data with links
  const weeklyTopics = [
    { title: 'Introduction to Physical AI', weeks: 'Weeks 1–2', link: '/week-1-2' },
    { title: 'ROS 2 Fundamentals', weeks: 'Weeks 3–5', link: '/week-3-5' },
    { title: 'Robot Simulation with Gazebo', weeks: 'Weeks 6–7', link: '/week-6-7' },
    { title: 'NVIDIA Isaac Platform', weeks: 'Weeks 8–10', link: '/week-8-10' },
    { title: 'Humanoid Robot Development', weeks: 'Weeks 11–12', link: '/week-11-12' },
    { title: 'Conversational Robotics', weeks: 'Week 13', link: '/week-13' },
  ];

  return (
    <section className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={clsx('hero__title', styles.mainTitle)}>
              {siteConfig.title}
            </Heading>
            <p className={clsx('hero__subtitle', styles.subtitle)}>
              {siteConfig.tagline}
            </p>

            {/* Weekly Topics Strip */}
            <div className={styles.topicsStrip}>
              <h3 className={styles.topicsTitle}>Course Structure</h3>
              <div className={styles.topicsContainer}>
                {weeklyTopics.map((topic, index) => (
                  <Link
                    key={index}
                    to={topic.link}
                    className={styles.topicTagLink}
                  >
                    <div className={styles.topicTag}>
                      <span className={styles.topicTitle}>{topic.title}</span>
                      <span className={styles.topicWeeks}>{topic.weeks}</span>
                    </div>
                  </Link>
                ))}
              </div>
            </div>

            <div className={styles.buttons}>
              <Link className="button button--secondary button--lg" to="/docs/intro">
                Start Learning - 5min ⏱️
              </Link>
            </div>
          </div>

          {/* Futuristic decorative elements */}
          <div className={styles.futuristicElements}>
            <div className={styles.gridPattern}></div>
            <div className={styles.circle1}></div>
            <div className={styles.circle2}></div>
            <div className={styles.wavePattern}></div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default HomepageHero;
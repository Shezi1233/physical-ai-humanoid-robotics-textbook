import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week35(): JSX.Element {
  return (
    <Layout title="Week 3-5: ROS 2 Fundamentals" description="ROS 2 Fundamentals content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Weeks 3-5" startWeek={3} endWeek={5} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
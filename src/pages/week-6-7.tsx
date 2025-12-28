import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week67(): JSX.Element {
  return (
    <Layout title="Week 6-7: Robot Simulation with Gazebo" description="Robot Simulation with Gazebo content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Weeks 6-7" startWeek={6} endWeek={7} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
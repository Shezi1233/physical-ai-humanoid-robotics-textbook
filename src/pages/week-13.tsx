import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week13(): JSX.Element {
  return (
    <Layout title="Week 13: Conversational Robotics" description="Conversational Robotics content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Week 13" startWeek={13} endWeek={13} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
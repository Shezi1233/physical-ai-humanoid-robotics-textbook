import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week810(): JSX.Element {
  return (
    <Layout title="Week 8-10: NVIDIA Isaac Platform" description="NVIDIA Isaac Platform content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Weeks 8-10" startWeek={8} endWeek={10} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
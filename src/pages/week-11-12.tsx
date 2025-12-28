import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week1112(): JSX.Element {
  return (
    <Layout title="Week 11-12: Humanoid Robot Development" description="Humanoid Robot Development content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Weeks 11-12" startWeek={11} endWeek={12} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
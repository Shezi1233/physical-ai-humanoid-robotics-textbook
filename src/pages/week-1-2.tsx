import React from 'react';
import Layout from '@theme/Layout';
import WeekTopics from '@site/src/components/WeekTopics';

export default function Week12(): JSX.Element {
  return (
    <Layout title="Week 1-2: Introduction to Physical AI" description="Introduction to Physical AI content">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <WeekTopics weekRange="Weeks 1-2" startWeek={1} endWeek={2} />
          </div>
        </div>
      </div>
    </Layout>
  );
}
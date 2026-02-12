import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import RagChatbot from '../components/RagChatbot/RagChatbot';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => <RagChatbot />}
      </BrowserOnly>
    </>
  );
}
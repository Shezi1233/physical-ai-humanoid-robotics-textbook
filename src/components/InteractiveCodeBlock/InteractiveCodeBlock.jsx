import React from 'react';
import clsx from 'clsx';
import styles from './InteractiveCodeBlock.module.css';

// A simple interactive code block component for educational purposes
const InteractiveCodeBlock = ({ children, language = 'python', title = 'Code Example' }) => {
  const [copied, setCopied] = React.useState(false);

  const copyToClipboard = () => {
    navigator.clipboard.writeText(children);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className={styles.codeBlockContainer}>
      <div className={styles.codeHeader}>
        <span className={styles.codeTitle}>{title}</span>
        <button
          className={styles.copyButton}
          onClick={copyToClipboard}
          aria-label="Copy code to clipboard"
        >
          {copied ? '✓ Copied!' : 'Copy'}
        </button>
      </div>
      <pre className={styles.codeBlock}>
        <code className={`language-${language}`}>
          {children}
        </code>
      </pre>
    </div>
  );
};

export default InteractiveCodeBlock;
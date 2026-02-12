import React, { useState, useEffect, useRef } from 'react';
import styles from './RagChatbot.module.css';

interface Citation {
  source_file: string;
  section_title: string;
  excerpt?: string;
}

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  citations?: Citation[];
  isError?: boolean;
}

interface ChatResponse {
  answer: string;
  citations: Citation[];
  session_id: string;
  metadata?: {
    model_used: string;
    latency_ms: number;
    fallback_used: boolean;
    chunks_retrieved: number;
  };
}

const PRODUCTION_BACKEND = 'https://shezi1344-physical-ai-chatbot.hf.space';

const getBackendUrl = () => {
  if (typeof window !== 'undefined') {
    return (window as any).BACKEND_URL || PRODUCTION_BACKEND;
  }
  return PRODUCTION_BACKEND;
};

const STORAGE_KEY = 'physical-ai-session-id';

const generateSessionId = () => {
  if (typeof window === 'undefined') {
    return `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
  try {
    let sessionId = localStorage.getItem(STORAGE_KEY);
    if (!sessionId) {
      sessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem(STORAGE_KEY, sessionId);
    }
    return sessionId;
  } catch {
    return `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
};

const WELCOME_MESSAGE: Message = {
  id: '1',
  content: "Hi! I'm your Physical AI & Robotics assistant. Ask me anything from the textbook — ROS 2, Gazebo, Isaac Sim, humanoid navigation, and more.",
  role: 'assistant',
  timestamp: new Date(),
};

const QUICK_PROMPTS = [
  'What is ROS 2?',
  'Explain URDF for humanoids',
  'How does Nav2 work?',
];

/* ── Icons ─────────────────────────────────────────────── */

const BotIcon = () => (
  <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
    <rect x="3" y="4" width="18" height="16" rx="3" />
    <circle cx="8.5" cy="10" r="1.5" fill="currentColor" stroke="none" />
    <circle cx="15.5" cy="10" r="1.5" fill="currentColor" stroke="none" />
    <path d="M9 15h6" />
  </svg>
);

const SendIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M22 2 11 13" />
    <path d="M22 2 15 22 11 13 2 9z" />
  </svg>
);

const CloseIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M18 6 6 18" />
    <path d="M6 6l12 12" />
  </svg>
);

const BookIcon = () => (
  <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M4 19.5v-15A2.5 2.5 0 0 1 6.5 2H20v20H6.5a2.5 2.5 0 0 1 0-5H20" />
  </svg>
);

const AlertIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="10" />
    <line x1="12" y1="8" x2="12" y2="12" />
    <line x1="12" y1="16" x2="12.01" y2="16" />
  </svg>
);

/* ── Toast Component ───────────────────────────────────── */

const Toast: React.FC<{ message: string; onDismiss: () => void }> = ({ message, onDismiss }) => {
  useEffect(() => {
    const timer = setTimeout(onDismiss, 5000);
    return () => clearTimeout(timer);
  }, [onDismiss]);

  return (
    <div className={styles.toast}>
      <AlertIcon />
      <span>{message}</span>
      <button onClick={onDismiss} className={styles.toastClose} aria-label="Dismiss">
        <CloseIcon />
      </button>
    </div>
  );
};

/* ── Typing Indicator ──────────────────────────────────── */

const TypingIndicator: React.FC = () => (
  <div className={styles.typingRow}>
    <div className={styles.avatar}>
      <BotIcon />
    </div>
    <div className={styles.typingBubble}>
      <span /><span /><span />
    </div>
  </div>
);

/* ── Message Bubble ────────────────────────────────────── */

const MessageBubble: React.FC<{ message: Message; isLatest: boolean }> = ({ message, isLatest }) => {
  const isUser = message.role === 'user';
  return (
    <div
      className={`${styles.messageRow} ${isUser ? styles.messageRowUser : styles.messageRowBot}`}
    >
      {!isUser && (
        <div className={styles.avatar}>
          <BotIcon />
        </div>
      )}
      <div className={styles.messageBubbleWrap}>
        <div className={`${styles.bubble} ${isUser ? styles.bubbleUser : styles.bubbleBot} ${message.isError ? styles.bubbleError : ''}`}>
          {message.content}
        </div>
        {message.citations && message.citations.length > 0 && (
          <div className={styles.citationBlock}>
            <div className={styles.citationLabel}>
              <BookIcon /> Sources
            </div>
            {message.citations.map((c, i) => (
              <div key={i} className={styles.citationItem}>
                <span className={styles.citationTitle}>{c.section_title}</span>
                {c.excerpt && (
                  <span className={styles.citationExcerpt}>
                    {c.excerpt.length > 120 ? c.excerpt.slice(0, 120) + '…' : c.excerpt}
                  </span>
                )}
              </div>
            ))}
          </div>
        )}
        <span className={styles.timestamp}>
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </span>
      </div>
    </div>
  );
};

/* ── Main Chatbot Component ────────────────────────────── */

const RagChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([WELCOME_MESSAGE]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState('');
  const [backendUrl, setBackendUrl] = useState('');
  const [toast, setToast] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  useEffect(() => {
    setSessionId(generateSessionId());
    setBackendUrl(getBackendUrl());
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  useEffect(() => {
    if (isOpen) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [isOpen]);

  const sendMessage = async (query: string) => {
    if (!query.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: query.trim(),
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 30000);

    try {
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: query.trim(), session_id: sessionId }),
        signal: controller.signal,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData?.error?.message || errorData?.detail || `Server error (${response.status})`);
      }

      const data: ChatResponse = await response.json();

      if (data.session_id && data.session_id !== sessionId) {
        setSessionId(data.session_id);
        try { localStorage.setItem(STORAGE_KEY, data.session_id); } catch {}
      }

      setMessages(prev => [
        ...prev,
        {
          id: (Date.now() + 1).toString(),
          content: data.answer || 'No response received.',
          role: 'assistant',
          timestamp: new Date(),
          citations: data.citations,
        },
      ]);
    } catch (error) {
      const isTimeout = error instanceof DOMException && error.name === 'AbortError';
      const errorMsg = isTimeout
        ? 'Request timed out. The server may be waking up — please try again.'
        : error instanceof Error ? error.message : 'Connection failed';
      setToast(errorMsg);
      setMessages(prev => [
        ...prev,
        {
          id: (Date.now() + 1).toString(),
          content: isTimeout
            ? 'The request timed out. Hugging Face Spaces can take a moment to wake up — please try again shortly.'
            : 'Something went wrong. Please check that the backend is running and try again.',
          role: 'assistant',
          timestamp: new Date(),
          isError: true,
        },
      ]);
    } finally {
      clearTimeout(timeout);
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage(inputValue);
    }
  };

  const handleQuickPrompt = (prompt: string) => {
    sendMessage(prompt);
  };

  const showQuickPrompts = messages.length <= 1 && !isLoading;

  /* ── Closed State: FAB ─────────────────────────────── */

  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className={styles.fab}
        aria-label="Open Physical AI Assistant"
      >
        <BotIcon />
      </button>
    );
  }

  /* ── Open State: Chat Window ───────────────────────── */

  return (
    <>
      {toast && <Toast message={toast} onDismiss={() => setToast(null)} />}

      <div className={styles.overlay} onClick={() => setIsOpen(false)} />

      <div className={styles.chatWindow} role="dialog" aria-label="Physical AI Assistant">
        {/* Header */}
        <header className={styles.header}>
          <div className={styles.headerLeft}>
            <div className={styles.headerIcon}>
              <BotIcon />
            </div>
            <div>
              <h3 className={styles.headerTitle}>Physical AI Assistant</h3>
              <p className={styles.headerSub}>Humanoid Robotics Textbook</p>
            </div>
          </div>
          <button onClick={() => setIsOpen(false)} className={styles.headerClose} aria-label="Close chat">
            <CloseIcon />
          </button>
        </header>

        {/* Messages */}
        <div className={styles.messagesArea}>
          {messages.map((msg, idx) => (
            <MessageBubble key={msg.id} message={msg} isLatest={idx === messages.length - 1} />
          ))}
          {isLoading && <TypingIndicator />}
          <div ref={messagesEndRef} />

          {/* Quick Prompts */}
          {showQuickPrompts && (
            <div className={styles.quickPrompts}>
              <span className={styles.quickLabel}>Try asking:</span>
              {QUICK_PROMPTS.map((p) => (
                <button key={p} onClick={() => handleQuickPrompt(p)} className={styles.quickBtn}>
                  {p}
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Input */}
        <div className={styles.inputArea}>
          <div className={styles.inputWrap}>
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics, ROS 2, simulation…"
              rows={1}
              className={styles.textInput}
              disabled={isLoading}
            />
            <button
              onClick={() => sendMessage(inputValue)}
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendBtn}
              aria-label="Send message"
            >
              <SendIcon />
            </button>
          </div>
          <p className={styles.footerNote}>
            Answers from the Physical AI & Humanoid Robotics textbook
          </p>
        </div>
      </div>
    </>
  );
};

export default RagChatbot;

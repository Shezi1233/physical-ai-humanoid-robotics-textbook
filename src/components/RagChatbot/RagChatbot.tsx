import React, { useState, useEffect } from 'react';
import styles from './RagChatbot.module.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const RagChatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your robotics education assistant. Ask me anything about ROS 2, Gazebo, Isaac, or Vision-Language-Action systems.',
      role: 'assistant',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // In a real implementation, this would call the backend API
      // For now, we'll simulate a response
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Simulated response - in real implementation, this would come from the backend
      const responseMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: `I received your question: "${inputValue}". In a real implementation, this would be processed by our RAG system to provide an accurate answer based on the robotics documentation.`,
        role: 'assistant',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, responseMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.chatHeader}>
        <h3>Robotics Education Assistant</h3>
        <p>Powered by RAG (Retrieval Augmented Generation)</p>
      </div>

      <div className={styles.chatMessages}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${styles[message.role]}`}
          >
            <div className={styles.messageContent}>
              {message.content}
            </div>
            <div className={styles.timestamp}>
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className={styles.message + ' ' + styles.assistant}>
            <div className={styles.typingIndicator}>
              <div className={styles.dot}></div>
              <div className={styles.dot}></div>
              <div className={styles.dot}></div>
            </div>
          </div>
        )}
      </div>

      <div className={styles.chatInput}>
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about robotics..."
          rows={2}
          disabled={isLoading}
          className={styles.textInput}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputValue.trim() || isLoading}
          className={styles.sendButton}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div className={styles.chatInfo}>
        <p>This chatbot uses RAG technology to provide accurate answers based on the robotics documentation.</p>
      </div>
    </div>
  );
};

export default RagChatbot;
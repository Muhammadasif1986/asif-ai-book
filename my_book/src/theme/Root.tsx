import React from 'react';
import { ChatWidget } from '../components/ChatWidget/ChatWidget';
import { SelectionHandler } from '../components/SelectionHandler/SelectionHandler';

// Create a wrapper component that includes both the chat widget and selection handler
const ChatIntegrationWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return (
    <>
      {children}
      <SelectionHandler />
      <ChatWidget
        apiUrl={process.env.REACT_APP_API_URL || '/api/v1'}
        bookId={process.env.REACT_APP_BOOK_ID || 'asif-ai-book'}
      />
    </>
  );
};

// The Root component is the top-level wrapper for the entire Docusaurus app
const Root: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return <ChatIntegrationWrapper>{children}</ChatIntegrationWrapper>;
};

export default Root;
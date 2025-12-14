/**
 * Example integration of the ChatWidget with Docusaurus layout.
 * This component demonstrates how to properly integrate the chat widget
 * with the Docusaurus documentation site.
 */

import React, { useEffect } from 'react';
import ChatWidget from './ChatWidget';
import SelectionHandler from '../SelectionHandler/SelectionHandler';

const DocusaurusChatIntegration = ({ children }) => {
  // Initialize the chat widget when component mounts
  useEffect(() => {
    // Any initialization code can go here
    console.log('Chat widget integrated with Docusaurus layout');
  }, []);

  return (
    <>
      {/* Render the original Docusaurus content */}
      {children}

      {/* Add the selection handler to capture text selections */}
      <SelectionHandler
        onSelectionChange={(selectedText) => {
          // This function will be called whenever text is selected
          // The ChatWidget will automatically pick up the selection
          console.log('Text selected:', selectedText.substring(0, 50) + '...');
        }}
      />

      {/* Add the chat widget */}
      <ChatWidget
        apiUrl={process.env.REACT_APP_API_URL || '/api/v1'}
        bookId={process.env.REACT_APP_BOOK_ID || 'asif-ai-book'}
      />
    </>
  );
};

export default DocusaurusChatIntegration;
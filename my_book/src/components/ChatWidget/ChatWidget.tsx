/**
 * ChatWidget component for the RAG chatbot system.
 * Provides an interface for users to ask questions about the book content.
 */

import React, { useState, useEffect, useRef } from 'react';
import { FiSend, FiX, FiMessageSquare, FiMaximize2, FiMinimize2 } from 'react-icons/fi';
import { Message } from './Message';
import { InputArea } from './InputArea';
import { useChat } from '../../hooks/useChat';
import { useSelection } from '../../hooks/useSelection';
import { ChatApiResponse } from '../../types';

interface ChatWidgetProps {
  apiUrl?: string;
  bookId?: string;
  className?: string;
}

export const ChatWidget: React.FC<ChatWidgetProps> = ({
  apiUrl = '/api/v1',
  bookId = 'default-book',
  className = ''
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [sessionToken, setSessionToken] = useState<string>(() => {
    // Try to get session token from localStorage or create a new one
    const savedToken = localStorage.getItem('chat-session-token');
    if (savedToken) return savedToken;

    const newToken = `sess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem('chat-session-token', newToken);
    return newToken;
  });

  const { messages, sendMessage, isLoading, error } = useChat(apiUrl, sessionToken);
  const { selectedText: hookSelectedText, isSelected } = useSelection();
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Handle text selection changes
  useEffect(() => {
    if (isSelected) {
      setSelectedText(hookSelectedText);
      setShowSelectionButton(true);
    } else {
      setShowSelectionButton(false);
    }
  }, [hookSelectedText, isSelected]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (chatContainerRef.current) {
      chatContainerRef.current.scrollTop = chatContainerRef.current.scrollHeight;
    }
  }, [messages]);

  const handleSendMessage = async (question: string) => {
    if (!question.trim()) return;

    // Determine context type based on whether text is selected
    const contextType = selectedText ? 'selection' : 'full_book';
    const messageData = {
      question: question.trim(),
      book_id: bookId,
      session_token: sessionToken,
      ...(contextType === 'selection' && { selected_text: selectedText })
    };

    await sendMessage(messageData, contextType);
    // Clear selected text after sending
    if (contextType === 'selection') {
      setSelectedText('');
      setShowSelectionButton(false);
    }
  };

  const handleAskAboutSelection = () => {
    if (!selectedText) return;

    // Set the selected text as the input in the message area
    // This will be handled by the InputArea component
    setIsOpen(true);
  };

  const toggleWidget = () => {
    setIsOpen(!isOpen);
  };

  const toggleMaximize = () => {
    setIsMaximized(!isMaximized);
  };

  return (
    <div className={`font-sans ${className}`}>
      {/* Floating button to open chat */}
      {!isOpen && (
        <button
          onClick={toggleWidget}
          className="fixed bottom-6 right-6 bg-blue-600 text-white p-4 rounded-full shadow-lg hover:bg-blue-700 transition-colors z-50 flex items-center justify-center"
          aria-label="Open chat"
        >
          <FiMessageSquare className="w-6 h-6" />
        </button>
      )}

      {/* Selection button - appears when text is selected */}
      {showSelectionButton && !isOpen && (
        <button
          onClick={handleAskAboutSelection}
          className="fixed bottom-20 right-6 bg-green-600 text-white px-4 py-2 rounded-lg shadow-lg hover:bg-green-700 transition-colors z-50 flex items-center"
          aria-label="Ask about selected text"
        >
          <FiMessageSquare className="w-4 h-4 mr-2" />
          Ask about this
        </button>
      )}

      {/* Chat widget */}
      {isOpen && (
        <div
          className={`fixed z-50 rounded-lg shadow-xl border border-gray-200 overflow-hidden transition-all duration-300 ${
            isMaximized
              ? 'top-4 left-4 right-4 bottom-4'
              : 'bottom-6 right-6 w-[400px] h-[600px]'
          }`}
        >
          {/* Header */}
          <div className="bg-blue-600 text-white p-4 flex justify-between items-center">
            <div className="flex items-center">
              <FiMessageSquare className="w-5 h-5 mr-2" />
              <h2 className="font-semibold">Book Assistant</h2>
            </div>
            <div className="flex space-x-2">
              <button
                onClick={toggleMaximize}
                className="text-white hover:text-gray-200 focus:outline-none"
                aria-label={isMaximized ? "Minimize chat" : "Maximize chat"}
              >
                {isMaximized ? <FiMinimize2 /> : <FiMaximize2 />}
              </button>
              <button
                onClick={toggleWidget}
                className="text-white hover:text-gray-200 focus:outline-none"
                aria-label="Close chat"
              >
                <FiX />
              </button>
            </div>
          </div>

          {/* Chat container */}
          <div
            ref={chatContainerRef}
            className="bg-gray-50 h-[calc(100%-130px)] overflow-y-auto p-4"
          >
            {/* Welcome message */}
            {messages.length === 0 && (
              <div className="mb-4 text-center text-gray-600">
                <p>Hello! I'm your book assistant.</p>
                <p>You can ask me questions about the book content.</p>
                {selectedText && (
                  <p className="mt-2 text-sm bg-blue-100 p-2 rounded">
                    Selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                  </p>
                )}
              </div>
            )}

            {/* Messages */}
            {messages.map((msg, index) => (
              <Message
                key={index}
                message={msg}
                sender={msg.sender}
              />
            ))}

            {/* Loading indicator */}
            {isLoading && (
              <div className="flex items-center mb-4">
                <div className="bg-blue-100 rounded-full p-2 mr-3">
                  <div className="bg-blue-500 rounded-full w-2 h-2 animate-bounce"></div>
                </div>
                <div className="bg-blue-100 rounded-lg p-3">
                  <div className="flex space-x-1">
                    <div className="w-2 h-2 bg-blue-500 rounded-full animate-bounce"></div>
                    <div className="w-2 h-2 bg-blue-500 rounded-full animate-bounce delay-100"></div>
                    <div className="w-2 h-2 bg-blue-500 rounded-full animate-bounce delay-200"></div>
                  </div>
                </div>
              </div>
            )}

            {/* Error message */}
            {error && (
              <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <strong>Error:</strong> {error.message || 'An error occurred while processing your request.'}
              </div>
            )}
          </div>

          {/* Input area */}
          <div className="bg-white border-t border-gray-200 p-4">
            <InputArea
              onSendMessage={handleSendMessage}
              isLoading={isLoading}
              selectedText={selectedText}
              onClearSelection={() => {
                setSelectedText('');
                setShowSelectionButton(false);
              }}
            />
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
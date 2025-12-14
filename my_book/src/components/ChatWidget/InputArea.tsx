/**
 * InputArea component for the RAG chatbot system.
 * Handles user input for questions and message submission.
 */

import React, { useState, KeyboardEvent, useRef, useEffect } from 'react';
import { FiSend, FiX, FiInfo } from 'react-icons/fi';

interface InputAreaProps {
  onSendMessage: (message: string) => void;
  isLoading: boolean;
  selectedText?: string;
  onClearSelection?: () => void;
}

export const InputArea: React.FC<InputAreaProps> = ({
  onSendMessage,
  isLoading,
  selectedText,
  onClearSelection
}) => {
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Adjust textarea height based on content
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 150)}px`;
    }
  }, [inputValue]);

  const handleSubmit = () => {
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className="space-y-3">
      {/* Selected text indicator */}
      {selectedText && (
        <div className="bg-blue-50 border border-blue-200 rounded-lg p-2 text-sm">
          <div className="flex justify-between items-start mb-1">
            <div className="flex items-center text-blue-700 font-medium">
              <FiInfo className="mr-1" />
              <span>Using selected text</span>
            </div>
            {onClearSelection && (
              <button
                onClick={onClearSelection}
                className="text-blue-500 hover:text-blue-700"
                aria-label="Clear selection"
              >
                <FiX />
              </button>
            )}
          </div>
          <div className="text-gray-600 truncate">
            "{selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}"
          </div>
        </div>
      )}

      {/* Input area */}
      <div className="flex">
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={selectedText ? "Ask about the selected text..." : "Ask a question about the book..."}
          disabled={isLoading}
          rows={1}
          className="flex-1 border border-gray-300 rounded-l-lg p-3 resize-none focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed"
        />
        <button
          onClick={handleSubmit}
          disabled={!inputValue.trim() || isLoading}
          className={`bg-blue-600 text-white px-4 rounded-r-lg flex items-center justify-center ${
            (!inputValue.trim() || isLoading) ? 'opacity-50 cursor-not-allowed' : 'hover:bg-blue-700'
          }`}
          aria-label="Send message"
        >
          {isLoading ? (
            <div className="w-5 h-5 border-t-2 border-white border-solid rounded-full animate-spin"></div>
          ) : (
            <FiSend className="w-5 h-5" />
          )}
        </button>
      </div>

      {/* Hint text */}
      <div className="text-xs text-gray-500">
        Press Enter to send, Shift+Enter for new line
      </div>
    </div>
  );
};

export default InputArea;
/**
 * Message component for the RAG chatbot system.
 * Displays individual messages in the chat interface.
 */

import React from 'react';
import { FiUser, FiBot } from 'react-icons/fi';
import Markdown from 'react-markdown';
import { ChatMessage } from '../../types';

interface MessageProps {
  message: ChatMessage;
  sender: 'user' | 'bot';
}

export const Message: React.FC<MessageProps> = ({ message, sender }) => {
  const isUser = sender === 'user';

  return (
    <div className={`flex mb-4 ${isUser ? 'justify-end' : 'justify-start'}`}>
      <div
        className={`max-w-[80%] rounded-lg p-3 ${
          isUser
            ? 'bg-blue-500 text-white rounded-tr-none'
            : 'bg-gray-200 text-gray-800 rounded-tl-none'
        }`}
      >
        <div className="flex items-start">
          <div className="mr-2 mt-0.5">
            {isUser ? (
              <FiUser className="w-5 h-5" />
            ) : (
              <FiBot className="w-5 h-5 text-blue-600" />
            )}
          </div>

          <div className="flex-1">
            {message.type === 'text' ? (
              <Markdown
                className={`prose prose-sm ${isUser ? 'text-white' : 'text-gray-800'}`}
              >
                {message.content}
              </Markdown>
            ) : message.type === 'loading' ? (
              <div className="flex space-x-1">
                <div className="w-2 h-2 bg-current rounded-full animate-bounce"></div>
                <div className="w-2 h-2 bg-current rounded-full animate-bounce delay-100"></div>
                <div className="w-2 h-2 bg-current rounded-full animate-bounce delay-200"></div>
              </div>
            ) : (
              <div className="text-red-600">Unsupported message type</div>
            )}

            {message.citations && message.citations.length > 0 && (
              <div className={`mt-2 pt-2 border-t ${isUser ? 'border-blue-400' : 'border-gray-300'}`}>
                <div className="text-xs font-medium opacity-80 mb-1">Sources:</div>
                <ul className="text-xs space-y-1">
                  {message.citations.map((citation, index) => (
                    <li key={index} className="flex items-start">
                      <span className="mr-1">•</span>
                      <span>
                        {citation.title}{citation.section && ` (${citation.section})`}
                        {citation.page_number && `, p. ${citation.page_number}`}
                      </span>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        </div>

        {message.timestamp && (
          <div
            className={`text-xs mt-1 ${
              isUser ? 'text-blue-200' : 'text-gray-500'
            }`}
          >
            {new Date(message.timestamp).toLocaleTimeString([], {
              hour: '2-digit',
              minute: '2-digit'
            })}
          </div>
        )}
      </div>
    </div>
  );
};

export default Message;
/**
 * useChat hook for the RAG chatbot system.
 * Manages chat state, message history, and API interactions.
 */

import { useState, useCallback } from 'react';
import { ChatMessage, ChatApiResponse } from '../types';
import { chatApi } from '../services/chat-api';

export const useChat = (apiUrl: string, sessionToken: string) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const addMessage = useCallback((message: ChatMessage) => {
    setMessages(prev => [...prev, message]);
  }, []);

  const sendMessage = useCallback(async (messageData: any, contextType: 'full_book' | 'selection' = 'full_book') => {
    setIsLoading(true);
    setError(null);

    try {
      // Add user message to UI immediately
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        content: messageData.question,
        sender: 'user',
        timestamp: new Date().toISOString(),
        type: 'text',
        citations: []
      };
      addMessage(userMessage);

      // Call the API to get the response
      let response: ChatApiResponse;
      if (contextType === 'selection' && messageData.selected_text) {
        response = await chatApi.querySelection(apiUrl, {
          question: messageData.question,
          selected_text: messageData.selected_text,
          session_token: messageData.session_token
        });
      } else {
        response = await chatApi.queryBook(apiUrl, {
          question: messageData.question,
          session_token: messageData.session_token,
          book_id: messageData.book_id
        });
      }

      // Add bot response to messages
      const botMessage: ChatMessage = {
        id: Date.now().toString(),
        content: response.answer,
        sender: 'bot',
        timestamp: new Date().toISOString(),
        type: 'text',
        citations: response.citations || []
      };
      addMessage(botMessage);

      return response;
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err instanceof Error ? err : new Error('Failed to send message'));

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error while processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date().toISOString(),
        type: 'text',
        citations: []
      };
      addMessage(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [addMessage, apiUrl]);

  const clearChat = useCallback(() => {
    setMessages([]);
    setError(null);
  }, []);

  const loadHistory = useCallback(async () => {
    // In a real implementation, this would load chat history from an API
    // For now, we'll just return the current messages
    return messages;
  }, [messages]);

  return {
    messages,
    sendMessage,
    isLoading,
    error,
    clearChat,
    loadHistory
  };
};

export default useChat;
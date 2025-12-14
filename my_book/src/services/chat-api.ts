/**
 * API service for chat functionality in the RAG chatbot system.
 * Handles communication with the backend API endpoints.
 */

import { ChatApiResponse, QueryRequest, SelectionQueryRequest } from '../types';

class ChatApiService {
  /**
   * Query the entire book for information
   */
  async queryBook(apiUrl: string, requestData: QueryRequest): Promise<ChatApiResponse> {
    try {
      const response = await fetch(`${apiUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error querying book:', error);
      throw error;
    }
  }

  /**
   * Query based on selected text only
   */
  async querySelection(apiUrl: string, requestData: SelectionQueryRequest): Promise<ChatApiResponse> {
    try {
      const response = await fetch(`${apiUrl}/query/selection`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error querying selection:', error);
      throw error;
    }
  }

  /**
   * Get health status of the API
   */
  async healthCheck(apiUrl: string): Promise<{ status: string; timestamp: string; services: any }> {
    try {
      const response = await fetch(`${apiUrl}/health`, {
        method: 'GET',
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error checking health:', error);
      throw error;
    }
  }

  /**
   * Ingest book content into the system
   */
  async ingestBook(apiUrl: string, bookData: any): Promise<any> {
    try {
      const response = await fetch(`${apiUrl}/ingest`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(bookData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error ingesting book:', error);
      throw error;
    }
  }
}

export const chatApi = new ChatApiService();
export default chatApi;
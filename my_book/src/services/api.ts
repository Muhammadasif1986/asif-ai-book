/**
 * General API service for the RAG chatbot system.
 * Provides common API utilities and shared functionality.
 */

export interface ApiResponse<T = any> {
  success: boolean;
  data?: T;
  error?: string;
  message?: string;
  status: number;
}

export interface ApiConfig {
  baseUrl: string;
  timeout?: number;
  headers?: Record<string, string>;
}

class ApiService {
  private config: ApiConfig;

  constructor(config: ApiConfig) {
    this.config = {
      timeout: 30000, // Default 30-second timeout
      headers: {
        'Content-Type': 'application/json',
      },
      ...config,
    };
  }

  /**
   * Makes a GET request to the specified endpoint
   */
  async get<T = any>(endpoint: string, params?: Record<string, any>): Promise<ApiResponse<T>> {
    let url = `${this.config.baseUrl}${endpoint}`;

    if (params) {
      const searchParams = new URLSearchParams(params);
      url += `?${searchParams.toString()}`;
    }

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(url, {
        method: 'GET',
        headers: { ...this.config.headers },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      return this.handleResponse<T>(response);
    } catch (error) {
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }
      throw error;
    }
  }

  /**
   * Makes a POST request to the specified endpoint
   */
  async post<T = any>(endpoint: string, data?: any): Promise<ApiResponse<T>> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(`${this.config.baseUrl}${endpoint}`, {
        method: 'POST',
        headers: { ...this.config.headers },
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      return this.handleResponse<T>(response);
    } catch (error) {
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }
      throw error;
    }
  }

  /**
   * Makes a PUT request to the specified endpoint
   */
  async put<T = any>(endpoint: string, data?: any): Promise<ApiResponse<T>> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(`${this.config.baseUrl}${endpoint}`, {
        method: 'PUT',
        headers: { ...this.config.headers },
        body: data ? JSON.stringify(data) : undefined,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      return this.handleResponse<T>(response);
    } catch (error) {
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }
      throw error;
    }
  }

  /**
   * Makes a DELETE request to the specified endpoint
   */
  async delete<T = any>(endpoint: string): Promise<ApiResponse<T>> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(`${this.config.baseUrl}${endpoint}`, {
        method: 'DELETE',
        headers: { ...this.config.headers },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      return this.handleResponse<T>(response);
    } catch (error) {
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server');
      }
      throw error;
    }
  }

  /**
   * Handles the response from fetch and returns a standardized format
   */
  private async handleResponse<T>(response: Response): Promise<ApiResponse<T>> {
    const contentType = response.headers.get('content-type');

    let data: T | undefined;
    if (contentType && contentType.includes('application/json')) {
      data = await response.json();
    } else {
      // For non-JSON responses, we'll return the text content
      const text = await response.text();
      if (text) {
        try {
          data = JSON.parse(text) as T;
        } catch {
          // If it's not JSON, we won't include data in the response
        }
      }
    }

    if (!response.ok) {
      return {
        success: false,
        error: data ? (data as any).error || (data as any).message || 'Request failed' : 'Request failed',
        status: response.status,
      };
    }

    return {
      success: true,
      data,
      status: response.status,
    };
  }

  /**
   * Updates the base URL for all requests
   */
  setBaseUrl(baseUrl: string) {
    this.config = { ...this.config, baseUrl };
  }

  /**
   * Updates headers for all requests
   */
  setHeaders(headers: Record<string, string>) {
    this.config = { ...this.config, headers: { ...this.config.headers, ...headers } };
  }

  /**
   * Sets an authentication token for all subsequent requests
   */
  setAuthToken(token: string, type: 'Bearer' | 'Basic' = 'Bearer') {
    this.setHeaders({
      Authorization: `${type} ${token}`,
    });
  }
}

// Create a default instance
export const api = new ApiService({
  baseUrl: typeof window !== 'undefined' ? window.location.origin : 'http://localhost:8000',
});

export default api;
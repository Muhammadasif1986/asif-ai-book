/**
 * TypeScript types for the RAG chatbot system.
 */

// Chat message types
export interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'bot';
  timestamp: string;
  type: 'text' | 'loading' | 'error';
  citations?: Citation[];
}

export interface Citation {
  title: string;
  section: string;
  page_number?: number;
  relevance_score: number;
  text_preview: string;
}

// API request/response types
export interface QueryRequest {
  question: string;
  session_token: string;
  book_id?: string;
  max_results?: number;
}

export interface SelectionQueryRequest {
  question: string;
  selected_text: string;
  session_token: string;
  book_id?: string;
  max_results?: number;
}

export interface ChatApiResponse {
  answer: string;
  citations: Citation[];
  response_time_ms: number;
  context_type: 'full_book' | 'selection';
  retrieved_chunks?: RetrievedChunk[];
}

export interface RetrievedChunk {
  content: string;
  score: number;
  source: {
    title: string;
    section: string;
    page_number?: number;
    content_id: string;
    source_file: string;
    created_at: string;
  };
}

// Content chunk types
export interface ContentChunk {
  content_id: string;
  book_id: string;
  title: string;
  section: string;
  page_number?: number;
  chunk_index: number;
  original_text: string;
  source_file: string;
  created_at: string;
  metadata?: Record<string, any>;
}

// User session types
export interface UserSession {
  session_token: string;
  is_authenticated: boolean;
  user_id?: string;
  created_at: string;
  last_activity_at: string;
  expires_at?: string;
}

// Query session types
export interface UserQuerySession {
  id: string;
  session_token: string;
  user_id?: string;
  question: string;
  context_type: 'full_book' | 'chapter' | 'selection';
  selected_text?: string;
  retrieved_chunks?: RetrievedChunk[];
  answer: string;
  citations?: Citation[];
  response_time_ms?: number;
  created_at: string;
  updated_at: string;
}

// Book metadata types
export interface BookMetadata {
  id: string;
  title: string;
  author?: string;
  description?: string;
  version?: string;
  total_chunks: number;
  total_pages?: number;
  word_count?: number;
  ingestion_status: 'pending' | 'in_progress' | 'completed' | 'failed';
  ingestion_started_at?: string;
  ingestion_completed_at?: string;
  created_at: string;
  updated_at: string;
}

// Content embedding types
export interface ContentEmbedding {
  id: string;
  content_id: string;
  book_id: string;
  chunk_text: string;
  chunk_title?: string;
  chunk_section?: string;
  chunk_index: number;
  source_file?: string;
  embedding_status: 'pending' | 'processed' | 'failed';
  vector_id?: string;
  created_at: string;
  updated_at: string;
}

// API usage metrics types
export interface APIMetric {
  id: string;
  session_token?: string;
  endpoint: string;
  request_data?: string;
  response_time_ms?: number;
  status_code?: number;
  rate_limited: boolean;
  created_at: string;
}

// Error types
export interface ApiError {
  error: string;
  message: string;
  timestamp: string;
  details?: Record<string, any>;
}

// Configuration types
export interface AppConfig {
  apiUrl: string;
  bookId: string;
  maxSelectionLength: number;
  rateLimitRequests: number;
  rateLimitWindow: number;
}

// Health check types
export interface HealthStatus {
  status: 'healthy' | 'degraded' | 'unavailable';
  timestamp: string;
  services: {
    [service: string]: 'available' | 'unavailable' | 'degraded';
  };
  version: string;
  environment: string;
  response_time_ms: number;
  checks?: {
    unavailable_services: string[];
    unconfigured_services: string[];
  };
}
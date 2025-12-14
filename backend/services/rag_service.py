from typing import List, Dict, Any, Optional
import logging
import time
from datetime import datetime

from services.qdrant_service import QdrantService
from services.openrouter_service import OpenRouterService
from api.query import ChatApiResponse, Citation

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_service: QdrantService, openrouter_service: OpenRouterService):
        self.qdrant_service = qdrant_service
        self.openrouter_service = openrouter_service

    async def query_full_book(self, question: str, book_id: str, session_token: str,
                             max_results: int = 5) -> ChatApiResponse:
        """Query the full book content"""
        start_time = time.time()

        try:
            # Generate embedding for the question
            question_embedding = await self.openrouter_service.generate_embedding(question)

            # Search for relevant content in Qdrant
            search_results = await self.qdrant_service.search_similar(
                query_embedding=question_embedding,
                book_id=book_id,
                limit=max_results
            )

            # Generate response using the LLM
            context = [result for result in search_results]
            answer = await self.openrouter_service.generate_response(question, context)

            # Format citations
            citations = []
            for result in search_results:
                citations.append(Citation(
                    title=result.get("metadata", {}).get("title", "Unknown"),
                    section=result.get("metadata", {}).get("section", "Unknown"),
                    page_number=result.get("metadata", {}).get("page_number"),
                    relevance_score=result.get("score", 0.0),
                    text_preview=result.get("text", "")[:200] + "..." if len(result.get("text", "")) > 200 else result.get("text", "")
                ))

            response_time = round((time.time() - start_time) * 1000, 2)

            return ChatApiResponse(
                answer=answer,
                citations=citations,
                response_time_ms=response_time,
                context_type="full_book",
                retrieved_chunks=search_results
            )
        except Exception as e:
            logger.error(f"Error in query_full_book: {str(e)}")
            raise

    async def query_selection(self, question: str, selected_text: str, book_id: str,
                             session_token: str, max_results: int = 5) -> ChatApiResponse:
        """Query based on selected text only"""
        start_time = time.time()

        try:
            # Combine question and selected text for context
            context_text = f"Selected text: {selected_text}\n\nQuestion: {question}"

            # Generate embedding for the combined context
            context_embedding = await self.openrouter_service.generate_embedding(context_text)

            # Search for relevant content in Qdrant (focused on the selected text's context)
            search_results = await self.qdrant_service.search_similar(
                query_embedding=context_embedding,
                book_id=book_id,
                limit=max_results
            )

            # Generate response using the LLM
            context = [result for result in search_results]
            answer = await self.openrouter_service.generate_response(question, context)

            # Format citations
            citations = []
            for result in search_results:
                citations.append(Citation(
                    title=result.get("metadata", {}).get("title", "Unknown"),
                    section=result.get("metadata", {}).get("section", "Unknown"),
                    page_number=result.get("metadata", {}).get("page_number"),
                    relevance_score=result.get("score", 0.0),
                    text_preview=result.get("text", "")[:200] + "..." if len(result.get("text", "")) > 200 else result.get("text", "")
                ))

            response_time = round((time.time() - start_time) * 1000, 2)

            return ChatApiResponse(
                answer=answer,
                citations=citations,
                response_time_ms=response_time,
                context_type="selection",
                retrieved_chunks=search_results
            )
        except Exception as e:
            logger.error(f"Error in query_selection: {str(e)}")
            raise
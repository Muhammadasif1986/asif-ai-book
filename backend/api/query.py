from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import logging
import os
from datetime import datetime

# Import service classes
from services.rag_service import RAGService
from services.qdrant_service import QdrantService
from services.openrouter_service import OpenRouterService

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services
qdrant_service = QdrantService(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY"),
    collection_name=os.getenv("QDRANT_COLLECTION", "book_content")
)
openrouter_service = OpenRouterService(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    model=os.getenv("OPENROUTER_MODEL", "openai/gpt-3.5-turbo")
)
rag_service = RAGService(qdrant_service, openrouter_service)

class QueryRequest(BaseModel):
    question: str
    session_token: str
    book_id: Optional[str] = "default-book"
    max_results: Optional[int] = 5

class SelectionQueryRequest(BaseModel):
    question: str
    selected_text: str
    session_token: str
    book_id: Optional[str] = "default-book"
    max_results: Optional[int] = 5

class Citation(BaseModel):
    title: str
    section: str
    page_number: Optional[int] = None
    relevance_score: float
    text_preview: str

class ChatApiResponse(BaseModel):
    answer: str
    citations: List[Citation]
    response_time_ms: float
    context_type: str
    retrieved_chunks: Optional[List[dict]] = None

@router.post("/query", response_model=ChatApiResponse)
async def query_book(request: QueryRequest):
    try:
        response = await rag_service.query_full_book(
            question=request.question,
            book_id=request.book_id,
            session_token=request.session_token,
            max_results=request.max_results
        )
        return response
    except Exception as e:
        logger.error(f"Error querying book: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/query/selection", response_model=ChatApiResponse)
async def query_selection(request: SelectionQueryRequest):
    try:
        response = await rag_service.query_selection(
            question=request.question,
            selected_text=request.selected_text,
            book_id=request.book_id,
            session_token=request.session_token,
            max_results=request.max_results
        )
        return response
    except Exception as e:
        logger.error(f"Error querying selection: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
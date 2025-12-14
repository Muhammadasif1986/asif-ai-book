from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging
import os
from datetime import datetime

# Import service classes
from services.ingestion_service import IngestionService
from services.qdrant_service import QdrantService

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services
qdrant_service = QdrantService(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY"),
    collection_name=os.getenv("QDRANT_COLLECTION", "book_content")
)
ingestion_service = IngestionService(qdrant_service)

class IngestionRequest(BaseModel):
    book_id: str
    title: str
    content: str
    source_file: str
    metadata: Optional[Dict[str, Any]] = None

class IngestionResponse(BaseModel):
    status: str
    message: str
    chunks_processed: int
    processing_time_ms: float

class IngestBookContentRequest(BaseModel):
    book_id: str
    title: str
    chapters: List[Dict[str, Any]]  # Each chapter has title, content, etc.

@router.post("/ingest", response_model=IngestionResponse)
async def ingest_content(request: IngestionRequest):
    try:
        result = await ingestion_service.ingest_content(
            book_id=request.book_id,
            title=request.title,
            content=request.content,
            source_file=request.source_file,
            metadata=request.metadata
        )
        return result
    except Exception as e:
        logger.error(f"Error ingesting content: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/ingest/book", response_model=IngestionResponse)
async def ingest_book_content(request: IngestBookContentRequest):
    try:
        result = await ingestion_service.ingest_book_content(
            book_id=request.book_id,
            title=request.title,
            chapters=request.chapters
        )
        return result
    except Exception as e:
        logger.error(f"Error ingesting book content: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/ingest/docusaurus")
async def ingest_docusaurus_docs():
    """
    Special endpoint to extract and ingest content from Docusaurus docs
    """
    try:
        result = await ingestion_service.ingest_docusaurus_docs()
        return result
    except Exception as e:
        logger.error(f"Error ingesting Docusaurus docs: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
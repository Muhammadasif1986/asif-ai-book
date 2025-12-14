#!/usr/bin/env python3
"""
Ingestion script to extract and embed book content from Docusaurus docs
"""
import asyncio
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent))

from services.ingestion_service import IngestionService
from services.qdrant_service import QdrantService

async def main():
    # Initialize services
    qdrant_service = QdrantService(
        url=os.getenv("QDRANT_URL", "http://localhost:6333"),
        api_key=os.getenv("QDRANT_API_KEY"),
        collection_name=os.getenv("QDRANT_COLLECTION", "book_content")
    )

    ingestion_service = IngestionService(qdrant_service)

    print("Starting Docusaurus docs ingestion...")

    try:
        result = await ingestion_service.ingest_docusaurus_docs()
        print(f"Ingestion completed successfully!")
        print(f"Status: {result['status']}")
        print(f"Message: {result['message']}")
        print(f"Chunks processed: {result['chunks_processed']}")
        print(f"Processing time: {result['processing_time_ms']}ms")
        print(f"Files processed: {result.get('files_processed', 0)}")
    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
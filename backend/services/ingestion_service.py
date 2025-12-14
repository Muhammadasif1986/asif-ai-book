from typing import List, Dict, Any, Optional
import logging
import os
import time
import asyncio
from pathlib import Path
import re

from services.qdrant_service import QdrantService
from services.openrouter_service import OpenRouterService

logger = logging.getLogger(__name__)

class IngestionService:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.openrouter_service = OpenRouterService(
            api_key=os.getenv("OPENROUTER_API_KEY"),
            model=os.getenv("OPENROUTER_MODEL", "openai/gpt-3.5-turbo")
        )

    def _extract_content_from_markdown(self, content: str) -> List[Dict[str, Any]]:
        """Extract sections from markdown content"""
        sections = []

        # Split content by headings
        lines = content.split('\n')
        current_section = {
            'title': 'Introduction',
            'content': '',
            'section': 'introduction'
        }

        for line in lines:
            # Check if this line is a heading
            heading_match = re.match(r'^(#{1,6})\s+(.+)', line)
            if heading_match:
                # Save the previous section if it has content
                if current_section['content'].strip():
                    sections.append(current_section)

                # Start a new section
                level = len(heading_match.group(1))
                title = heading_match.group(2).strip()
                current_section = {
                    'title': title,
                    'content': f'# {title}\n',
                    'section': title.lower().replace(' ', '_').replace('-', '_')
                }
            else:
                current_section['content'] += line + '\n'

        # Add the last section
        if current_section['content'].strip():
            sections.append(current_section)

        return sections

    def _chunk_text(self, text: str, max_chunk_size: int = 1000) -> List[str]:
        """Split text into chunks of appropriate size"""
        sentences = re.split(r'[.!?]+\s+', text)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk) + len(sentence) < max_chunk_size:
                current_chunk += sentence + ". "
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + ". "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    async def ingest_content(self, book_id: str, title: str, content: str,
                           source_file: str, metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Ingest a single content piece"""
        start_time = time.time()

        try:
            # Extract sections from content
            sections = self._extract_content_from_markdown(content)
            total_chunks = 0

            for section in sections:
                # Chunk the section content
                chunks = self._chunk_text(section['content'])

                for i, chunk in enumerate(chunks):
                    # Generate embedding for the chunk
                    embedding = await self.openrouter_service.generate_embedding(chunk)

                    # Prepare metadata for storage
                    chunk_metadata = {
                        'title': section['title'],
                        'section': section['section'],
                        'source_file': source_file,
                        'chunk_index': i,
                        'book_id': book_id,
                        'created_at': time.time(),
                        **(metadata or {})
                    }

                    # Store in Qdrant
                    content_id = f"{book_id}_{section['section']}_chunk_{i}_{int(time.time())}"
                    success = await self.qdrant_service.store_embedding(
                        content_id=content_id,
                        text=chunk,
                        embedding=embedding,
                        book_id=book_id,
                        metadata=chunk_metadata
                    )

                    if success:
                        total_chunks += 1
                    else:
                        logger.warning(f"Failed to store chunk {content_id}")

            processing_time = round((time.time() - start_time) * 1000, 2)

            return {
                "status": "success",
                "message": f"Successfully ingested {total_chunks} chunks",
                "chunks_processed": total_chunks,
                "processing_time_ms": processing_time
            }
        except Exception as e:
            logger.error(f"Error ingesting content: {str(e)}")
            raise

    async def ingest_book_content(self, book_id: str, title: str,
                                 chapters: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Ingest an entire book with multiple chapters"""
        start_time = time.time()
        total_chunks = 0

        try:
            for chapter in chapters:
                chapter_title = chapter.get('title', 'Untitled Chapter')
                chapter_content = chapter.get('content', '')
                source_file = chapter.get('source_file', f"{book_id}_chapter")

                result = await self.ingest_content(
                    book_id=book_id,
                    title=chapter_title,
                    content=chapter_content,
                    source_file=source_file,
                    metadata={'chapter_title': chapter_title}
                )

                total_chunks += result['chunks_processed']

            processing_time = round((time.time() - start_time) * 1000, 2)

            return {
                "status": "success",
                "message": f"Successfully ingested book with {total_chunks} total chunks",
                "chunks_processed": total_chunks,
                "processing_time_ms": processing_time
            }
        except Exception as e:
            logger.error(f"Error ingesting book content: {str(e)}")
            raise

    async def ingest_docusaurus_docs(self) -> Dict[str, Any]:
        """Extract and ingest content from Docusaurus docs directory"""
        start_time = time.time()
        total_chunks = 0

        try:
            # Define the docs path relative to the backend
            docs_path = Path('../../my_book/docs')  # Adjust path as needed

            if not docs_path.exists():
                # Try alternative path
                docs_path = Path('../my_book/docs')
                if not docs_path.exists():
                    docs_path = Path('/mnt/d/asif-ai-book/my_book/docs')

            if not docs_path.exists():
                raise FileNotFoundError(f"Docs directory not found at {docs_path}")

            # Get all markdown files
            md_files = list(docs_path.rglob('*.md'))

            for md_file in md_files:
                try:
                    # Read the markdown file
                    content = md_file.read_text(encoding='utf-8')

                    # Create a book_id based on the file path
                    relative_path = md_file.relative_to(docs_path.parent)
                    book_id = f"docusaurus_{relative_path.parent.name}_{relative_path.stem}".replace('/', '_').replace('\\', '_')

                    # Ingest the content
                    result = await self.ingest_content(
                        book_id=book_id,
                        title=md_file.stem,
                        content=content,
                        source_file=str(relative_path)
                    )

                    total_chunks += result['chunks_processed']
                    logger.info(f"Processed {md_file.name}: {result['chunks_processed']} chunks")

                except Exception as e:
                    logger.error(f"Error processing file {md_file}: {str(e)}")
                    continue  # Continue with other files

            processing_time = round((time.time() - start_time) * 1000, 2)

            return {
                "status": "success",
                "message": f"Successfully ingested Docusaurus docs with {total_chunks} total chunks from {len(md_files)} files",
                "chunks_processed": total_chunks,
                "processing_time_ms": processing_time,
                "files_processed": len(md_files)
            }
        except Exception as e:
            logger.error(f"Error ingesting Docusaurus docs: {str(e)}")
            raise
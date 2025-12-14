from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self, url: str, api_key: Optional[str] = None, collection_name: str = "book_content"):
        self.client = QdrantClient(url=url, api_key=api_key, prefer_grpc=False)
        self.collection_name = collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with appropriate vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Default OpenAI embedding size
                        distance=models.Distance.COSINE
                    )
                )

                # Create payload index for faster filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="book_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {str(e)}")
            raise

    async def store_embedding(self, content_id: str, text: str, embedding: List[float],
                             book_id: str, metadata: Dict[str, Any] = None) -> bool:
        """Store a single embedding in Qdrant"""
        try:
            # Prepare payload
            payload = {
                "content_id": content_id,
                "text": text,
                "book_id": book_id,
                "created_at": datetime.now().isoformat(),
                **(metadata or {})
            }

            # Store in Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=content_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
            return True
        except Exception as e:
            logger.error(f"Error storing embedding: {str(e)}")
            return False

    async def search_similar(self, query_embedding: List[float], book_id: str,
                           limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar content in Qdrant"""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_id",
                            match=models.MatchValue(value=book_id)
                        )
                    ]
                ),
                limit=limit,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "text": result.payload.get("text", ""),
                    "score": result.score,
                    "content_id": result.payload.get("content_id", ""),
                    "book_id": result.payload.get("book_id", ""),
                    "metadata": {k: v for k, v in result.payload.items()
                                if k not in ["text", "content_id", "book_id", "created_at"]}
                })

            return formatted_results
        except Exception as e:
            logger.error(f"Error searching similar content: {str(e)}")
            return []

    async def batch_store_embeddings(self, embeddings_data: List[Dict[str, Any]]) -> bool:
        """Store multiple embeddings in batch"""
        try:
            points = []
            for data in embeddings_data:
                content_id = data.get("content_id") or str(uuid.uuid4())
                payload = {
                    "content_id": content_id,
                    "text": data["text"],
                    "book_id": data["book_id"],
                    "created_at": datetime.now().isoformat(),
                    **(data.get("metadata", {}))
                }

                points.append(
                    models.PointStruct(
                        id=content_id,
                        vector=data["embedding"],
                        payload=payload
                    )
                )

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            logger.error(f"Error storing batch embeddings: {str(e)}")
            return False

    async def get_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve content by its ID"""
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id],
                with_payload=True,
                with_vectors=False
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "text": record.payload.get("text", ""),
                    "content_id": record.payload.get("content_id", ""),
                    "book_id": record.payload.get("book_id", ""),
                    "metadata": record.payload
                }
            return None
        except Exception as e:
            logger.error(f"Error retrieving content by ID: {str(e)}")
            return None
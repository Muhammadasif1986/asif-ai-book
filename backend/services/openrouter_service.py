import openai
from typing import List, Dict, Any, Optional
import logging
import os
import time

logger = logging.getLogger(__name__)

class OpenRouterService:
    def __init__(self, api_key: str, model: str = "openai/gpt-3.5-turbo"):
        self.api_key = api_key
        self.model = model
        # Set OpenRouter API base and key
        openai.api_key = api_key
        openai.base_url = "https://openrouter.ai/api/v1"

    async def generate_response(self, prompt: str, context: List[Dict[str, Any]] = None) -> str:
        """Generate response using OpenRouter API"""
        try:
            # Prepare context for the prompt
            context_str = ""
            if context:
                context_str = "Context:\n" + "\n".join([
                    f"- {item['text'][:200]}..." for item in context
                ]) + "\n\n"

            full_prompt = f"{context_str}Question: {prompt}\n\nPlease provide a helpful answer based on the context provided. If the context doesn't contain the information needed, please say so."

            response = openai.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a helpful assistant that answers questions based on provided context from a book. Be concise and accurate."
                    },
                    {
                        "role": "user",
                        "content": full_prompt
                    }
                ],
                temperature=0.3,
                max_tokens=1000
            )

            return response.choices[0].message.content.strip()
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenRouter's compatible embedding API"""
        try:
            # Using OpenAI-compatible embedding endpoint
            response = openai.embeddings.create(
                model="text-embedding-ada-002",  # Standard OpenAI embedding model
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts"""
        try:
            # For now, process one by one - in production, this could be optimized
            embeddings = []
            for text in texts:
                embedding = await self.generate_embedding(text)
                embeddings.append(embedding)
            return embeddings
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {str(e)}")
            raise
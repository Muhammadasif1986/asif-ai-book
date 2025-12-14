from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import logging
from datetime import datetime

# Import API routers
from api.health import router as health_router
from api.query import router as query_router
from api.ingestion import router as ingestion_router

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Asif AI Book RAG API",
    description="API for the AI-native book platform with RAG chatbot",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(query_router, prefix="/api/v1", tags=["query"])
app.include_router(ingestion_router, prefix="/api/v1", tags=["ingestion"])

@app.get("/")
async def root():
    return {"message": "Asif AI Book RAG API", "version": "1.0.0"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )
from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Any
import time
import os

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, str]
    version: str
    environment: str
    response_time_ms: float

@router.get("/health", response_model=HealthResponse)
async def health_check():
    start_time = time.time()

    # Check services status
    services_status = {
        "qdrant": "available",  # Placeholder - would check actual connection
        "postgres": "available",  # Placeholder - would check actual connection
        "openrouter": "available"  # Placeholder - would check actual connection
    }

    # Calculate response time
    response_time = round((time.time() - start_time) * 1000, 2)

    return HealthResponse(
        status="healthy",
        timestamp=str(int(time.time())),
        services=services_status,
        version="1.0.0",
        environment=os.getenv("ENVIRONMENT", "development"),
        response_time_ms=response_time
    )
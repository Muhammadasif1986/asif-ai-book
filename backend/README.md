# Asif AI Book Backend

Backend API for the AI-native book platform with RAG chatbot functionality.

## Features

- FastAPI-based REST API
- RAG (Retrieval Augmented Generation) capabilities
- Qdrant vector database integration
- OpenRouter LLM integration
- Full-book and selection-based querying

## Prerequisites

- Python 3.9+
- Qdrant Cloud account (for vector storage)
- OpenRouter API key (for LLM access)

## Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Configuration

1. Copy `.env.example` to `.env`:
   ```bash
   cp .env.example .env
   ```
2. Update the values in `.env` with your actual configuration:
   - `QDRANT_URL`: Your Qdrant Cloud cluster URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `OPENROUTER_API_KEY`: Your OpenRouter API key

## Running Locally

```bash
uvicorn app.main:app --reload
```

The API will be available at `http://localhost:8000`.

## Running with Docker

```bash
docker-compose up -d
```

## API Endpoints

- `GET /` - Root endpoint
- `GET /api/v1/health` - Health check
- `POST /api/v1/query` - Full-book query
- `POST /api/v1/query/selection` - Selection-based query
- `POST /api/v1/ingest` - Content ingestion
- `POST /api/v1/ingest/book` - Book content ingestion
- `POST /api/v1/ingest/docusaurus` - Docusaurus docs ingestion

## Environment Variables

- `PORT` - Port to run the server on (default: 8000)
- `ENVIRONMENT` - Environment name (default: development)
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `QDRANT_COLLECTION` - Qdrant collection name (default: book_content)
- `OPENROUTER_API_KEY` - OpenRouter API key
- `OPENROUTER_MODEL` - OpenRouter model to use (default: openai/gpt-3.5-turbo)

## Deployment

### To Cloud Platforms

The backend is designed to be deployed to cloud platforms like:

- AWS Elastic Beanstalk
- Google Cloud Run
- Azure Container Instances
- Railway
- Render

Make sure to set the required environment variables in your deployment platform.

### Using Docker

Build and push the Docker image to your container registry, then deploy to your preferred container platform.

## Ingestion

To ingest your book content:

1. Use the `/api/v1/ingest/docusaurus` endpoint to automatically extract and embed content from your Docusaurus docs
2. Or use `/api/v1/ingest` or `/api/v1/ingest/book` for manual content ingestion

## Architecture

- FastAPI for the web framework
- Qdrant for vector storage and similarity search
- OpenRouter for LLM interactions
- Pydantic for data validation
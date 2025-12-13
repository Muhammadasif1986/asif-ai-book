# Quickstart: Project Setup

## Prerequisites
- Python 3.11+
- pip package manager
- Git

## Installation

1. Install Cookiecutter:
   ```bash
   pip install cookiecutter
   ```

2. Generate a new project:
   ```bash
   cookiecutter path/to/project-template
   ```

3. Follow the prompts to provide project details:
   - `project_name`: Your project's name
   - `project_description`: Brief description
   - `version`: Initial version (default: 0.1.0)
   - `author`: Your name
   - `email`: Contact email
   - `license`: License type (default: MIT)
   - `include_docker`: Include Docker configuration? (default: y)
   - `include_ci`: Include CI/CD configuration? (default: y)
   - `include_docs`: Include documentation structure? (default: y)

## Project Structure

After generation, your project will include:

```
my-new-project/
├── src/                 # Source code
│   ├── __init__.py
│   └── main.py
├── tests/               # Test files
│   ├── __init__.py
│   └── test_main.py
├── docs/                # Documentation
│   ├── README.md
│   └── setup-guide.md
├── .env.example         # Example environment variables
├── .gitignore           # Git ignore rules
├── docker-compose.yml   # Docker configuration
├── Dockerfile           # Docker build file
├── pyproject.toml       # Project configuration
├── requirements.txt     # Python dependencies
├── pytest.ini           # Test configuration
└── README.md            # Project documentation
```

## Getting Started

1. Navigate to your new project:
   ```bash
   cd my-new-project
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Create your environment file:
   ```bash
   cp .env.example .env
   # Edit .env with your specific values
   ```

5. Run tests to verify setup:
   ```bash
   pytest
   ```

6. Start development:
   ```bash
   python src/main.py
   ```

## Configuration

Environment-specific settings are managed through `.env` files. Copy `.env.example` to `.env` and update values as needed for your environment. Add environment-specific configurations to the appropriate sections while maintaining security best practices.
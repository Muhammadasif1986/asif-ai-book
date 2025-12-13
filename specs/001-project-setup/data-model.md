# Data Model: Project Setup

## Project Template Entity

**Fields:**
- `project_name`: String - Name of the new project (user input)
- `project_description`: String - Brief description of the project (user input)
- `version`: String - Initial version of the project (default: "0.1.0")
- `author`: String - Author/owner of the project (user input)
- `email`: String - Contact email for the project (user input)
- `license`: String - License type for the project (default: "MIT")
- `python_version`: String - Target Python version (default: "^3.11")
- `include_docker`: Boolean - Whether to include Docker configuration (default: true)
- `include_ci`: Boolean - Whether to include CI/CD configuration (default: true)
- `include_docs`: Boolean - Whether to include documentation structure (default: true)

**Validation Rules:**
- `project_name` must follow Python package naming conventions (alphanumeric, underscores, hyphens)
- `email` must be a valid email format
- `license` must be a recognized open source license
- `python_version` must be a valid Python version specification

## Configuration Entity

**Fields:**
- `environment`: String - Environment name (development, staging, production)
- `config_variables`: Map<String, String> - Key-value pairs of configuration variables
- `secure_variables`: List<String> - Names of variables that should not be committed

**Validation Rules:**
- Environment must be one of the predefined values
- Configuration variables must follow naming conventions
- Secure variables must be documented in .env.example but not in .env

## Documentation Entity

**Fields:**
- `readme_content`: String - Main README content with project-specific details
- `setup_guide`: String - Step-by-step setup instructions
- `contribution_guide`: String - Guidelines for contributing to the project
- `architecture_doc`: String - High-level architecture overview

**Validation Rules:**
- All documentation must be in Markdown format
- Links and references must be valid
- Documentation must be updatable with project-specific details
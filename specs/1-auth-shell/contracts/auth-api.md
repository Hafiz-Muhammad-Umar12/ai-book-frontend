# API Contracts: Auth Shell

## Backend Authentication API (FastAPI)

This document outlines the expected API contract for the authentication backend, which will be implemented using FastAPI (Python). The frontend `authClient` will interact with this API at `http://localhost:8000`.

### General API Principles
- **Base URL**: `http://localhost:8000`
- **Authentication Method**: Token-based (e.g., JWT) is expected, handled by `better-auth` library.
- **Error Handling**: Standard HTTP status codes and JSON error responses.

### Expected Endpoints (Illustrative - actual endpoints determined by Better-Auth FastAPI integration)

#### 1. User Login
- **Endpoint**: `POST /login` (or similar, as defined by `better-auth`)
- **Description**: Authenticates a user and issues session tokens.
- **Request Body (Example)**:
  ```json
  {
    "username": "string",
    "password": "string"
  }
  ```
- **Response Body (Example - Success)**:
  ```json
  {
    "access_token": "string",
    "token_type": "bearer"
  }
  ```
- **Response Body (Example - Error)**:
  ```json
  {
    "detail": "Invalid credentials"
  }
  ```

#### 2. User Logout
- **Endpoint**: `POST /logout` (or similar)
- **Description**: Invalidates the current user session/token.
- **Request Body**: (May be empty or include a token for specific invalidation)
- **Response**: `204 No Content` or `200 OK`

#### 3. Get Current User / Session Status
- **Endpoint**: `GET /users/me` (or similar)
- **Description**: Retrieves information about the currently authenticated user.
- **Headers**: `Authorization: Bearer <access_token>`
- **Response Body (Example)**:
  ```json
  {
    "id": "uuid",
    "username": "string",
    "email": "string"
  }
  ```

### Versioning Strategy
- API versioning will follow standard practices, potentially via URL prefix (e.g., `/v1/login`) or header-based versioning, to be determined by the `better-auth` FastAPI implementation.

### OpenAPI Specification
- The definitive API contract will be provided by the FastAPI backend's automatically generated OpenAPI (Swagger) documentation, accessible at `/docs` or `/redoc` when the backend is running.
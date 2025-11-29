# Data Model: Auth Shell

## Key Entities

### SessionProvider
- **Description**: React context provider from Better-Auth for managing user sessions.
- **Fields**: (Implicitly manages session state, no explicit fields defined in spec)
- **Relationships**: Provides session context to child components within the Docusaurus frontend.

### authClient
- **Description**: An instance of the Better-Auth client configured for the backend.
- **Fields**:
  - `baseURL`: `http://localhost:8000` (as per FR-002)
- **Relationships**: Used by the `SessionProvider` to interact with the Better-Auth backend for authentication operations.
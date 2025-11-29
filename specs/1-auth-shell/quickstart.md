# Quickstart Guide: Auth Shell Implementation

This guide provides a quick overview of how to get the Auth Shell feature up and running for both the Docusaurus frontend and FastAPI backend.

## 1. Prerequisites

- Node.js and npm (for Docusaurus frontend)
- Python 3.11+ and pip (for FastAPI backend)
- Basic understanding of Docusaurus and FastAPI

## 2. Backend Setup (FastAPI)

Ensure your FastAPI backend is running and providing the authentication endpoints, typically on `http://localhost:8000`. The `better-auth` library should be integrated into your FastAPI application.

*Refer to the backend's specific documentation or implementation for detailed setup instructions and how to start the FastAPI server.*

## 3. Frontend Setup (Docusaurus)

### Step 3.1: Swizzle the Root Component

If you haven't already, swizzle the Docusaurus `Root` component to wrap your application with the `SessionProvider`.

```bash
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

This command will create `frontend/src/theme/Root.tsx`.

### Step 3.2: Integrate SessionProvider

Modify `frontend/src/theme/Root.tsx` to include the `SessionProvider` from `better-auth/react`.

First, install the necessary package:

```bash
npm install @better-auth/react
```

Then, update `frontend/src/theme/Root.tsx` (or similar root component) to wrap your app with `SessionProvider`:

```typescript
// frontend/src/theme/Root.tsx

import React from 'react';
import { SessionProvider } from '@better-auth/react';
import { createAuthClient } from '../lib/auth-client'; // We'll create this next

// Default AuthClient setup (you might want to make this configurable)
const authClient = createAuthClient({
  baseURL: 'http://localhost:8000',
});

function Root({ children }) {
  return (
    <SessionProvider authClient={authClient}>
      {children}
    </SessionProvider>
  );
}

export default Root;
```

### Step 3.3: Create `auth-client.ts`

Create a new file `frontend/src/lib/auth-client.ts` (or similar) to define your `createAuthClient` function. This client will point to your FastAPI backend.

```typescript
// frontend/src/lib/auth-client.ts

import { createAuthClient as createBetterAuthClient } from '@better-auth/react';

export function createAuthClient(options) {
  return createBetterAuthClient({
    baseURL: options.baseURL,
    // Add other configuration options as needed, e.g., storage, token handling
  });
}
```

### Step 3.4: Run the Docusaurus Frontend

Navigate to your frontend directory and start the Docusaurus development server:

```bash
cd frontend
npm start
```

The Docusaurus application should now be running with the `SessionProvider` active, attempting to connect to your backend at `http://localhost:8000`.
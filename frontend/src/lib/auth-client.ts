import { createAuthClient } from "./mock-auth";

const authClient = createAuthClient({
  baseURL: "https://ai-book-backend.vercel.app",
});

export default authClient;

import { createAuthClient } from "./mock-auth";

const authClient = createAuthClient({
  baseURL: "http://localhost:8000",
});

export default authClient;

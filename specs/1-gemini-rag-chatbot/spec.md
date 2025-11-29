# Feature Specification: Gemini-Powered RAG Chatbot

**Feature Branch**: `1-gemini-rag-chatbot`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: """ Feature: Gemini-Powered RAG Chatbot

Requirements:

Dependencies: Ensure openai and qdrant-client are installed.

Endpoint: Create POST /chat in backend/main.py.

Input Schema:

JSON

{
  "query": "How do I install ROS 2?",
  "context": "Optional selected text from the book"
}
RAG Logic:

Vector Search: Use QdrantClient to find chunks relevant to the query. (Mock this return if Qdrant URL is invalid/empty).

AI Generation: Initialize AsyncOpenAI with the Gemini Bridge Pattern (Base URL: https://generativelanguage.googleapis.com/v1beta/openai/).

System Prompt: "You are a Physical AI Teaching Assistant. Answer using the provided context."

Model: Use gemini-2.0-flash.

Output: Return {"response": "AI answer..."}.
"""

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask a Question with Context (Priority: P1)

As a user, I want to ask a question related to the Physical AI textbook and optionally provide selected text as context, so that the chatbot can provide a relevant and informed answer based on the book's content.

**Why this priority**: This is the core functionality of a RAG chatbot – answering questions based on provided knowledge.

**Independent Test**: Can be fully tested by sending a query with and without context to the `/chat` endpoint and verifying the AI's response draws from the provided context or vector search results.

**Acceptance Scenarios**:

1.  **Given** the chatbot is running, **When** I send a query "How do I install ROS 2?" without context, **Then** the chatbot returns a response based on the Qdrant vector search results.
2.  **Given** the chatbot is running, **When** I send a query "What are ROS 2 Nodes?" with context "In ROS 2, a Node is an executable process that performs a specific task.", **Then** the chatbot returns a response that incorporates the provided context.

---

### Edge Cases

- What happens when an empty query is sent?
- How does the system handle a QdrantClient connection failure (beyond mocking)?
- What is the behavior when the provided context is irrelevant to the query?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST provide an interface for chat interactions.
- **FR-002**: The `/chat` endpoint MUST accept a JSON input with `query` (string) and `context` (optional string) fields.
- **FR-003**: The system MUST retrieve relevant information from a knowledge base based on the `query`.
- **FR-004**: The system MUST provide fallback behavior if the knowledge base is unavailable.
- **FR-005**: The system MUST use an AI model to generate responses.
- **FR-006**: The AI generation MUST use the system prompt: "You are a Physical AI Teaching Assistant. Answer using the provided context."
- **FR-007**: The `/chat` endpoint MUST return a JSON output with a `response` field containing the AI's answer.

### Key Entities

-   **Query**: The natural language question posed by the user.
-   **Context**: Optional additional text provided by the user to guide the AI's response.
-   **Vector Embeddings/Chunks**: Text segments from the textbook, converted into numerical vectors for similarity search.
-   **AI Response**: The generated answer from the Gemini model.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The `/chat` endpoint responds to a query within 3 seconds.
-   **SC-002**: 90% of user queries (with or without context) receive relevant answers that directly address the question based on the provided context or vector search results.
-   **SC-003**: The chatbot provides responses powered by a large language model.

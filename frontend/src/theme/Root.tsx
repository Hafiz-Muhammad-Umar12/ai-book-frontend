import React from 'react';
import { SessionProvider } from "../lib/mock-auth";
import authClient from "../lib/auth-client";
import ChatWidget from '../components/ChatWidget';

export default function Root({children}) {
  return (
    <SessionProvider client={authClient}>
      {children}
      <ChatWidget />
    </SessionProvider>
  );
}
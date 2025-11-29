import React, { createContext, useContext, useState, useEffect } from 'react';

// 1. Create Context
const SessionContext = createContext({
  data: { user: { name: "Judge", hasGPU: undefined } }, // Undefined GPU triggers the modal
  error: null
});

// 2. Export Provider
export const SessionProvider = ({ children, client }) => {
  return (
    <SessionContext.Provider value={{ data: { user: { name: "Judge", hasGPU: undefined } }, error: null }}>
      {children}
    </SessionContext.Provider>
  );
};

// 3. Export Hook
export const useSession = () => {
  return useContext(SessionContext);
};

// 4. Export Client Helper
export const createAuthClient = (config) => ({
  signIn: async () => console.log("Mock Sign In"),
  signOut: async () => console.log("Mock Sign Out")
});
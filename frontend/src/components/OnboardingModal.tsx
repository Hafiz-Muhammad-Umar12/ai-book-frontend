import React from 'react';
import { useSession } from "../lib/mock-auth";

const OnboardingModal = () => {
  const { session } = useSession();

  if (!session || session.user?.hasGPU) {
    return null; // Don't show modal if not logged in or if user has GPU
  }

  return (
    <div className="onboarding-modal-overlay">
      <div className="onboarding-modal">
        <h2>Onboarding Modal</h2>
        <p>Do you have an NVIDIA GPU?</p>
        <input type="checkbox" id="hasNvidiaGpu" name="hasNvidiaGpu" />
        {/* Content will be added in subsequent tasks */}
      </div>
    </div>
  );
};

export default OnboardingModal;

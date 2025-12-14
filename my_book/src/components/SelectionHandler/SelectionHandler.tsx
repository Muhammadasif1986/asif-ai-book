/**
 * SelectionHandler component for the RAG chatbot system.
 * Provides global text selection tracking by using the useSelection hook.
 * This component should be mounted globally in the app to enable selection tracking.
 */

import React from 'react';
import { useSelection } from '../../hooks/useSelection';

// This component doesn't need to render anything; it just uses the hook
// which sets up the global event listeners
export const SelectionHandler: React.FC = () => {
  // Using the hook here ensures the global event listeners are set up
  const { selectedText, isSelected } = useSelection();

  return (
    <div data-testid="selection-handler" style={{ display: 'none' }}>
      {/* This component doesn't render anything visible, it just ensures the useSelection hook is active globally */}
    </div>
  );
};

export default SelectionHandler;
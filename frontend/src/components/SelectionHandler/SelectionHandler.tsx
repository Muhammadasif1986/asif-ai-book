/**
 * SelectionHandler component for the RAG chatbot system.
 * Handles text selection functionality and integrates with the chat system.
 */

import React, { useEffect, useRef } from 'react';

interface SelectionHandlerProps {
  onSelectionChange: (selectedText: string) => void;
}

export const SelectionHandler: React.FC<SelectionHandlerProps> = ({ onSelectionChange }) => {
  const selectionRef = useRef<string>('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim() || '';

      // Only trigger if there's actual text selected (not just clicking)
      if (selectedText !== selectionRef.current) {
        selectionRef.current = selectedText;
        onSelectionChange(selectedText);
      }
    };

    // Listen for mouseup and touchend events to capture text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    // Also listen for selectionchange for more comprehensive coverage
    document.addEventListener('selectionchange', handleSelection);

    // Cleanup event listeners on unmount
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('selectionchange', handleSelection);
    };
  }, [onSelectionChange]);

  return (
    <div data-testid="selection-handler" style={{ display: 'none' }}>
      {/* This component doesn't render anything visible, it just handles selection events */}
    </div>
  );
};

export default SelectionHandler;
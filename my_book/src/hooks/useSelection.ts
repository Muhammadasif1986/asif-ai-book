/**
 * useSelection hook for the RAG chatbot system.
 * Manages text selection state and provides selection-related functionality.
 */

import { useState, useEffect } from 'react';

export interface SelectionState {
  selectedText: string;
  isSelected: boolean;
  selectionRect?: DOMRect;
}

export const useSelection = (): SelectionState => {
  const [selectionState, setSelectionState] = useState<SelectionState>({
    selectedText: '',
    isSelected: false,
    selectionRect: undefined
  });

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim() || '';

      // Only update if there's a change in selection
      if (selectedText !== selectionState.selectedText) {
        let rect: DOMRect | undefined;

        if (selection && selection.rangeCount > 0) {
          const range = selection.getRangeAt(0);
          rect = range.getBoundingClientRect();
        }

        setSelectionState({
          selectedText,
          isSelected: selectedText.length > 0,
          selectionRect: rect
        });
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
  }, [selectionState.selectedText]);

  return selectionState;
};

export default useSelection;
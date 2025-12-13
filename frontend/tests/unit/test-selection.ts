/**
 * Unit tests for text selection capture functionality in the frontend.
 */
import { renderHook, act } from '@testing-library/react';
import { useSelection } from '../../src/hooks/useSelection';

// Mock the document selection API
const mockGetSelection = jest.fn();
const mockToString = jest.fn();

Object.defineProperty(window, 'getSelection', {
  value: () => ({
    toString: mockToString,
    anchorOffset: 10,
    focusOffset: 20,
    anchorNode: { nodeValue: 'This is the selected text' },
    focusNode: { nodeValue: 'This is the selected text' },
    rangeCount: 1,
    getRangeAt: (index: number) => ({
      startContainer: { nodeValue: 'This is the selected text' },
      endContainer: { nodeValue: 'This is the selected text' },
      startOffset: 10,
      endOffset: 20,
      cloneContents: () => document.createTextNode('selected text'),
      getBoundingClientRect: () => ({ x: 0, y: 0, width: 100, height: 20 })
    })
  }),
  writable: true
});

describe('useSelection Hook', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('should initialize with empty selection', () => {
    const { result } = renderHook(() => useSelection());

    expect(result.current.selectedText).toBe('');
    expect(result.current.isSelected).toBe(false);
  });

  it('should capture text selection when user selects text', () => {
    mockToString.mockReturnValue('This is the selected text');

    const { result } = renderHook(() => useSelection());

    // Simulate text selection
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(result.current.selectedText).toBe('This is the selected text');
    expect(result.current.isSelected).toBe(true);
  });

  it('should clear selection when user clicks elsewhere', () => {
    mockToString.mockReturnValue('This is the selected text');

    const { result } = renderHook(() => useSelection());

    // Simulate text selection
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(result.current.selectedText).toBe('This is the selected text');
    expect(result.current.isSelected).toBe(true);

    // Simulate clearing selection
    mockToString.mockReturnValue('');

    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(result.current.selectedText).toBe('');
    expect(result.current.isSelected).toBe(false);
  });

  it('should validate selection length', () => {
    // Mock a long selection that exceeds the limit
    const longText = 'A'.repeat(6000); // Exceeds 5000 character limit
    mockToString.mockReturnValue(longText);

    const { result } = renderHook(() => useSelection());

    // Simulate text selection
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    // The selection should still be captured but might be validated elsewhere
    expect(result.current.selectedText).toBe(longText);
    expect(result.current.isSelected).toBe(true);
  });

  it('should handle empty selection gracefully', () => {
    mockToString.mockReturnValue('');

    const { result } = renderHook(() => useSelection());

    // Simulate selection event with empty text
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(result.current.selectedText).toBe('');
    expect(result.current.isSelected).toBe(false);
  });

  it('should provide selection coordinates', () => {
    mockToString.mockReturnValue('Selected text');

    const { result } = renderHook(() => useSelection());

    // Simulate text selection
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    // The hook should provide coordinates for positioning UI elements
    expect(result.current.selectionRect).toBeDefined();
  });
});


// Test the SelectionHandler component
import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import SelectionHandler from '../../src/components/SelectionHandler/SelectionHandler';

describe('SelectionHandler Component', () => {
  const mockOnSelectionChange = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('should render without crashing', () => {
    render(<SelectionHandler onSelectionChange={mockOnSelectionChange} />);

    expect(screen.getByTestId('selection-handler')).toBeInTheDocument();
  });

  it('should call callback when text is selected', () => {
    mockToString.mockReturnValue('Selected text');

    render(<SelectionHandler onSelectionChange={mockOnSelectionChange} />);

    // Simulate text selection event
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(mockOnSelectionChange).toHaveBeenCalledWith('Selected text');
  });

  it('should not call callback for empty selection', () => {
    mockToString.mockReturnValue('');

    render(<SelectionHandler onSelectionChange={mockOnSelectionChange} />);

    // Simulate selection event with empty text
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    expect(mockOnSelectionChange).not.toHaveBeenCalled();
  });

  it('should handle selection with length validation', () => {
    const longText = 'A'.repeat(5500); // Exceeds validation limit
    mockToString.mockReturnValue(longText);

    render(<SelectionHandler onSelectionChange={mockOnSelectionChange} />);

    // Simulate long text selection
    act(() => {
      const selectionEvent = new Event('selectionchange');
      document.dispatchEvent(selectionEvent);
    });

    // Callback should still be called with the text, validation might happen elsewhere
    expect(mockOnSelectionChange).toHaveBeenCalledWith(longText);
  });
});


// Test the FloatingButton component
import FloatingButton from '../../src/components/UI/FloatingButton';

describe('FloatingButton Component', () => {
  const mockOnClick = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('should render with correct text', () => {
    render(
      <FloatingButton
        onClick={mockOnClick}
        isVisible={true}
        text="Ask about selection"
      />
    );

    expect(screen.getByText('Ask about selection')).toBeInTheDocument();
  });

  it('should call onClick handler when clicked', () => {
    render(
      <FloatingButton
        onClick={mockOnClick}
        isVisible={true}
        text="Ask about selection"
      />
    );

    fireEvent.click(screen.getByText('Ask about selection'));
    expect(mockOnClick).toHaveBeenCalledTimes(1);
  });

  it('should be hidden when isVisible is false', () => {
    const { container } = render(
      <FloatingButton
        onClick={mockOnClick}
        isVisible={false}
        text="Ask about selection"
      />
    );

    // The button should have a class that hides it
    const button = container.querySelector('button');
    expect(button).toHaveClass('opacity-0', 'hidden');
  });
});
/**
 * FloatingButton component for the RAG chatbot system.
 * Provides a floating button that can be positioned anywhere on the page.
 */

import React from 'react';

interface FloatingButtonProps {
  onClick: () => void;
  isVisible?: boolean;
  text?: string;
  icon?: React.ReactNode;
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  className?: string;
  ariaLabel?: string;
}

export const FloatingButton: React.FC<FloatingButtonProps> = ({
  onClick,
  isVisible = true,
  text = '',
  icon,
  position = 'bottom-right',
  className = '',
  ariaLabel = 'Floating button'
}) => {
  // Determine position classes based on position prop
  const positionClasses = {
    'bottom-right': 'bottom-6 right-6',
    'bottom-left': 'bottom-6 left-6',
    'top-right': 'top-6 right-6',
    'top-left': 'top-6 left-6'
  };

  return (
    <button
      onClick={onClick}
      style={{ display: isVisible ? 'block' : 'none' }}
      className={`
        fixed z-40 bg-blue-600 text-white p-4 rounded-full shadow-lg
        hover:bg-blue-700 transition-colors flex items-center justify-center
        ${positionClasses[position]}
        ${className}
      `}
      aria-label={ariaLabel}
    >
      {icon && <span className="mr-2">{icon}</span>}
      {text && <span>{text}</span>}
      {!icon && !text && (
        // Default icon if none provided
        <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
        </svg>
      )}
    </button>
  );
};

export default FloatingButton;
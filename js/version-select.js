/**
 * Enhanced Version Selector for Isaac ROS Workspace
 * 
 * Features:
 * - Modern ES6+ JavaScript
 * - Smooth animations and transitions
 * - Accessibility support
 * - Mobile-responsive
 * - Keyboard navigation
 * - Auto-detection of current version
 * - Loading states and error handling
 * - Local storage for user preferences
 */

class VersionSelector {
  constructor() {
    this.versions = {
      "latest": {
        url: "https://tng-blue.github.io/Isaac_ROS_WS/",
        label: "Latest",
        description: "Latest stable release",
        badge: "Stable"
      },
      "v2.1.0": {
        url: "https://tng-blue.github.io/Isaac_ROS_WS/v2.1.0/",
        label: "v2.1.0",
        description: "Version 2.1.0 - Isaac ROS 2.1 compatible",
        badge: "Current"
      },
      "v2.0.0": {
        url: "https://tng-blue.github.io/Isaac_ROS_WS/v2.0.0/",
        label: "v2.0.0",
        description: "Version 2.0.0 - Major release",
        badge: null
      },
      "dev": {
        url: "https://tng-blue.github.io/Isaac_ROS_WS/dev/",
        label: "Development",
        description: "Latest development build",
        badge: "Beta"
      }
    };
    
    this.currentVersion = this.detectCurrentVersion();
    this.container = null;
    this.select = null;
    this.isInitialized = false;
    
    // Configuration
    this.config = {
      enableKeyboardNavigation: true,
      enableLoadingStates: true,
      enableAnalytics: false,
      enableLocalStorage: true,
      animationDuration: 300,
      debounceDelay: 150
    };
    
    this.init();
  }
  
  /**
   * Initialize the version selector
   */
  init() {
    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => this.create());
    } else {
      this.create();
    }
  }
  
  /**
   * Detect current version based on URL
   */
  detectCurrentVersion() {
    const currentUrl = window.location.href;
    
    for (const [version, config] of Object.entries(this.versions)) {
      if (currentUrl.startsWith(config.url)) {
        return version;
      }
    }
    
    // Default to latest if no match found
    return 'latest';
  }
  
  /**
   * Create and inject the version selector
   */
  create() {
    try {
      this.createContainer();
      this.createSelect();
      this.attachEventListeners();
      this.applyStoredPreferences();
      this.animate();
      this.isInitialized = true;
      
      console.log('🚀 Isaac ROS Version Selector initialized successfully');
    } catch (error) {
      console.error('❌ Failed to initialize version selector:', error);
    }
  }
  
  /**
   * Create the main container element
   */
  createContainer() {
    this.container = document.createElement('div');
    this.container.className = 'version-select';
    this.container.setAttribute('role', 'navigation');
    this.container.setAttribute('aria-label', 'Version selector');
    this.container.setAttribute('title', 'Switch between different versions of the documentation');
    
    // Add current version badge if applicable
    const currentVersionConfig = this.versions[this.currentVersion];
    if (currentVersionConfig?.badge) {
      this.container.setAttribute('data-version', currentVersionConfig.badge);
      this.container.setAttribute('data-show-badge', 'true');
    }
    
    // Add screen reader text
    const srLabel = document.createElement('span');
    srLabel.className = 'sr-only';
    srLabel.textContent = 'Documentation version selector';
    this.container.appendChild(srLabel);
  }
  
  /**
   * Create the select dropdown element
   */
  createSelect() {
    this.select = document.createElement('select');
    this.select.setAttribute('aria-label', 'Select documentation version');
    this.select.setAttribute('title', 'Choose a different version');
    
    // Populate options
    for (const [version, config] of Object.entries(this.versions)) {
      const option = document.createElement('option');
      option.value = config.url;
      option.textContent = config.label;
      option.setAttribute('title', config.description);
      
      // Mark current version as selected
      if (version === this.currentVersion) {
        option.selected = true;
        option.setAttribute('aria-current', 'page');
      }
      
      this.select.appendChild(option);
    }
    
    this.container.appendChild(this.select);
    document.body.appendChild(this.container);
  }
  
  /**
   * Attach event listeners
   */
  attachEventListeners() {
    // Primary change event
    this.select.addEventListener('change', this.debounce(this.handleVersionChange.bind(this), this.config.debounceDelay));
    
    // Keyboard navigation
    if (this.config.enableKeyboardNavigation) {
      this.select.addEventListener('keydown', this.handleKeyNavigation.bind(this));
    }
    
    // Focus events for accessibility
    this.select.addEventListener('focus', this.handleFocus.bind(this));
    this.select.addEventListener('blur', this.handleBlur.bind(this));
    
    // Mouse events for enhanced UX
    this.container.addEventListener('mouseenter', this.handleMouseEnter.bind(this));
    this.container.addEventListener('mouseleave', this.handleMouseLeave.bind(this));
    
    // Window events
    window.addEventListener('resize', this.debounce(this.handleResize.bind(this), 250));
    window.addEventListener('beforeunload', this.handleBeforeUnload.bind(this));
    
    // Page visibility change
    document.addEventListener('visibilitychange', this.handleVisibilityChange.bind(this));
  }
  
  /**
   * Handle version change with enhanced UX
   */
  async handleVersionChange(event) {
    const selectedUrl = event.target.value;
    const currentUrl = window.location.href;
    
    // Prevent unnecessary navigation
    if (selectedUrl === currentUrl) {
      return;
    }
    
    try {
      // Show loading state
      if (this.config.enableLoadingStates) {
        this.showLoadingState();
      }
      
      // Store user preference
      if (this.config.enableLocalStorage) {
        this.storeVersionPreference(selectedUrl);
      }
      
      // Track analytics if enabled
      if (this.config.enableAnalytics) {
        this.trackVersionChange(selectedUrl);
      }
      
      // Add smooth transition
      await this.transitionOut();
      
      // Navigate to new version
      window.location.href = selectedUrl;
      
    } catch (error) {
      console.error('❌ Error during version change:', error);
      this.hideLoadingState();
      this.showErrorState('Failed to switch version. Please try again.');
    }
  }
  
  /**
   * Handle keyboard navigation
   */
  handleKeyNavigation(event) {
    switch (event.key) {
      case 'Enter':
      case ' ':
        event.preventDefault();
        this.select.click();
        break;
        
      case 'Escape':
        event.preventDefault();
        this.select.blur();
        break;
        
      case 'ArrowUp':
      case 'ArrowDown':
        // Let default behavior handle option navigation
        break;
        
      default:
        // Handle alphanumeric quick selection
        this.handleQuickSelection(event.key);
        break;
    }
  }
  
  /**
   * Handle quick selection via keyboard
   */
  handleQuickSelection(key) {
    const lowerKey = key.toLowerCase();
    const options = Array.from(this.select.options);
    
    // Find option starting with the pressed key
    const matchingOption = options.find(option => 
      option.textContent.toLowerCase().startsWith(lowerKey)
    );
    
    if (matchingOption) {
      this.select.value = matchingOption.value;
      matchingOption.selected = true;
    }
  }
  
  /**
   * Handle focus events
   */
  handleFocus(event) {
    this.container.classList.add('focused');
    this.container.setAttribute('aria-expanded', 'true');
    
    // Announce to screen readers
    this.announceToScreenReader('Version selector focused. Use arrow keys to navigate options.');
  }
  
  /**
   * Handle blur events
   */
  handleBlur(event) {
    this.container.classList.remove('focused');
    this.container.setAttribute('aria-expanded', 'false');
  }
  
  /**
   * Handle mouse enter
   */
  handleMouseEnter(event) {
    this.container.classList.add('hovered');
  }
  
  /**
   * Handle mouse leave
   */
  handleMouseLeave(event) {
    this.container.classList.remove('hovered');
  }
  
  /**
   * Handle window resize
   */
  handleResize() {
    // Adjust position for mobile if needed
    const isMobile = window.innerWidth <= 768;
    this.container.classList.toggle('mobile', isMobile);
  }
  
  /**
   * Handle before unload
   */
  handleBeforeUnload() {
    // Clean up any ongoing animations
    this.container.style.transition = 'none';
  }
  
  /**
   * Handle page visibility change
   */
  handleVisibilityChange() {
    if (document.hidden) {
      // Pause any animations when tab is not visible
      this.container.style.animationPlayState = 'paused';
    } else {
      // Resume animations when tab becomes visible
      this.container.style.animationPlayState = 'running';
    }
  }
  
  /**
   * Show loading state
   */
  showLoadingState() {
    this.container.classList.add('loading');
    this.select.disabled = true;
    
    // Update ARIA attributes
    this.container.setAttribute('aria-busy', 'true');
    this.announceToScreenReader('Loading new version...');
  }
  
  /**
   * Hide loading state
   */
  hideLoadingState() {
    this.container.classList.remove('loading');
    this.select.disabled = false;
    this.container.setAttribute('aria-busy', 'false');
  }
  
  /**
   * Show error state
   */
  showErrorState(message) {
    this.container.classList.add('error');
    this.announceToScreenReader(`Error: ${message}`);
    
    // Auto-hide error after 3 seconds
    setTimeout(() => {
      this.container.classList.remove('error');
    }, 3000);
  }
  
  /**
   * Animate entrance
   */
  animate() {
    // Initial state
    this.container.style.opacity = '0';
    this.container.style.transform = 'translateY(-20px) scale(0.9)';
    
    // Trigger animation
    requestAnimationFrame(() => {
      this.container.style.transition = `all ${this.config.animationDuration}ms cubic-bezier(0.4, 0, 0.2, 1)`;
      this.container.style.opacity = '0.95';
      this.container.style.transform = 'translateY(0) scale(1)';
    });
  }
  
  /**
   * Transition out animation
   */
  async transitionOut() {
    return new Promise(resolve => {
      this.container.style.transition = `all ${this.config.animationDuration}ms ease-in`;
      this.container.style.opacity = '0';
      this.container.style.transform = 'translateY(-10px) scale(0.95)';
      
      setTimeout(resolve, this.config.animationDuration);
    });
  }
  
  /**
   * Store version preference in localStorage
   */
  storeVersionPreference(url) {
    try {
      const preference = {
        url: url,
        timestamp: Date.now(),
        userAgent: navigator.userAgent
      };
      localStorage.setItem('isaac-docs-version-preference', JSON.stringify(preference));
    } catch (error) {
      console.warn('⚠️ Could not store version preference:', error);
    }
  }
  
  /**
   * Apply stored preferences
   */
  applyStoredPreferences() {
    if (!this.config.enableLocalStorage) return;
    
    try {
      const stored = localStorage.getItem('isaac-docs-version-preference');
      if (stored) {
        const preference = JSON.parse(stored);
        
        // Check if preference is recent (within 30 days)
        const isRecent = (Date.now() - preference.timestamp) < (30 * 24 * 60 * 60 * 1000);
        
        if (isRecent && preference.url && preference.url !== window.location.href) {
          // Show notification about available preferred version
          this.showVersionSuggestion(preference.url);
        }
      }
    } catch (error) {
      console.warn('⚠️ Could not apply stored preferences:', error);
    }
  }
  
  /**
   * Show version suggestion notification
   */
  showVersionSuggestion(suggestedUrl) {
    // Create a subtle notification
    const notification = document.createElement('div');
    notification.className = 'version-suggestion';
    notification.innerHTML = `
      <span>Your preferred version is available</span>
      <button onclick="window.location.href='${suggestedUrl}'">Switch</button>
      <button onclick="this.parentNode.remove()">Dismiss</button>
    `;
    
    // Style the notification
    notification.style.cssText = `
      position: fixed;
      bottom: 20px;
      right: 20px;
      background: var(--isaac-glass-bg);
      border: 1px solid var(--isaac-glass-border);
      backdrop-filter: var(--isaac-glass-blur);
      padding: 12px 16px;
      border-radius: 12px;
      box-shadow: var(--isaac-shadow-3);
      font-size: 0.875rem;
      z-index: 1040;
      max-width: 300px;
      opacity: 0;
      transform: translateY(20px);
      transition: all 0.3s ease;
    `;
    
    document.body.appendChild(notification);
    
    // Animate in
    requestAnimationFrame(() => {
      notification.style.opacity = '1';
      notification.style.transform = 'translateY(0)';
    });
    
    // Auto-dismiss after 10 seconds
    setTimeout(() => {
      if (notification.parentNode) {
        notification.style.opacity = '0';
        notification.style.transform = 'translateY(20px)';
        setTimeout(() => notification.remove(), 300);
      }
    }, 10000);
  }
  
  /**
   * Track version change for analytics
   */
  trackVersionChange(newUrl) {
    // Example: Google Analytics tracking
    if (typeof gtag !== 'undefined') {
      gtag('event', 'version_change', {
        'event_category': 'Documentation',
        'event_label': newUrl,
        'value': 1
      });
    }
    
    // Example: Custom analytics
    console.log('📊 Version change tracked:', {
      from: window.location.href,
      to: newUrl,
      timestamp: new Date().toISOString()
    });
  }
  
  /**
   * Announce message to screen readers
   */
  announceToScreenReader(message) {
    const announcement = document.createElement('div');
    announcement.setAttribute('aria-live', 'polite');
    announcement.setAttribute('aria-atomic', 'true');
    announcement.className = 'sr-only';
    announcement.textContent = message;
    
    document.body.appendChild(announcement);
    
    // Remove after announcement
    setTimeout(() => {
      document.body.removeChild(announcement);
    }, 1000);
  }
  
  /**
   * Debounce utility function
   */
  debounce(func, wait) {
    let timeout;
    return function executedFunction(...args) {
      const later = () => {
        clearTimeout(timeout);
        func(...args);
      };
      clearTimeout(timeout);
      timeout = setTimeout(later, wait);
    };
  }
  
  /**
   * Destroy the version selector
   */
  destroy() {
    if (this.container && this.container.parentNode) {
      this.container.parentNode.removeChild(this.container);
    }
    this.isInitialized = false;
    console.log('🗑️ Version selector destroyed');
  }
  
  /**
   * Update versions configuration
   */
  updateVersions(newVersions) {
    this.versions = { ...this.versions, ...newVersions };
    
    if (this.isInitialized) {
      // Recreate select options
      this.select.innerHTML = '';
      this.createSelect();
    }
  }
  
  /**
   * Get current version info
   */
  getCurrentVersionInfo() {
    return {
      version: this.currentVersion,
      config: this.versions[this.currentVersion],
      url: window.location.href
    };
  }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', function() {
  // Create global instance
  window.isaacVersionSelector = new VersionSelector();
  
  // Expose useful methods globally
  window.switchToVersion = function(version) {
    const versionConfig = window.isaacVersionSelector.versions[version];
    if (versionConfig) {
      window.location.href = versionConfig.url;
    } else {
      console.error('❌ Unknown version:', version);
    }
  };
  
  // Add keyboard shortcut (Ctrl/Cmd + Shift + V)
  document.addEventListener('keydown', function(event) {
    if ((event.ctrlKey || event.metaKey) && event.shiftKey && event.key === 'V') {
      event.preventDefault();
      if (window.isaacVersionSelector?.select) {
        window.isaacVersionSelector.select.focus();
      }
    }
  });
  
  console.log('🎉 Isaac ROS Documentation enhanced with professional version selector');
});
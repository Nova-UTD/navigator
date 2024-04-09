import LazyLoadedList from './lazy-loaded-list.svelte';

// Props for the LazyLoadedList component.
export type LazyLoadedListProps<T> = {
	// List of items to be rendered.
	items: T[];
	// Items per rendered chunk.
	itemsPerChunk: number;
	// ul element class.
	ulClasses?: string;
};

export { LazyLoadedList };

<script lang="ts" generics="T">
	import { onMount, onDestroy } from 'svelte';
	import { chunkArray, cn, seqID } from '$lib/utils';
	import type { LazyLoadedListProps } from '.';

	type $$Props = LazyLoadedListProps<T>;
	export let items: $$Props['items'] = [];
	export let itemsPerChunk: $$Props['itemsPerChunk'];
	export let ulClasses: $$Props['ulClasses'] = '';

	interface $$Slots {
		default: { item: T };
	}

	const listId = seqID();
	const endOfListID = listId + '-end-of-list';
	let displayedItems = items.slice(0, itemsPerChunk);

	// Group items into chunks.
	let chunks = chunkArray(items, itemsPerChunk);
	// The index of the last rendered chunk.
	let chunkIndex = 0;

	$: {
		chunks = chunkArray(items, itemsPerChunk);
		chunkIndex = 0;
		renderChunks();
	}

	function renderChunks() {
		if (chunkIndex > chunks.length) return;

		const endOfList = document.getElementById(endOfListID);
		// End of list should hide when there are no more chunks to render.
		if (!endOfList) return;

		const { top, bottom } = endOfList.getBoundingClientRect();

		if (top >= 0 && bottom <= window.innerHeight) {
			chunkIndex++;
			if (chunkIndex >= chunks.length) return;
			displayedItems = [...displayedItems, ...chunks[chunkIndex]];
		}
	}

	onMount(() => {
		renderChunks();
		document.getElementById(listId)?.addEventListener('scroll', renderChunks);
	});

	onDestroy(() => document.getElementById(listId)?.removeEventListener('scroll', renderChunks));
</script>

<ul class={cn('max-h-[400px] overflow-y-scroll', ulClasses)} id={listId}>
	{#each displayedItems as item}
		<li>
			<slot {item} />
		</li>
	{/each}
	<div id={endOfListID} hidden={chunkIndex === chunks.length} />
</ul>

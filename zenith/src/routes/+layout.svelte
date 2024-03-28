<script>
	import '../app.css';
	import { appWindow } from '@tauri-apps/api/window';
	import XIcon from 'lucide-svelte/icons/x';
	import MaximizeIcon from 'lucide-svelte/icons/maximize-2';
	import MinimizeIcon from 'lucide-svelte/icons/minimize-2';
	import MinusIcon from 'lucide-svelte/icons/minus';

	let isMaximized = false;
	$: appWindow.isMaximized().then((maximized) => {
		isMaximized = maximized;
	});
</script>

<div id="root" class="w-screen h-screen overflow-hidden flex flex-col">
	<div data-tauri-drag-region class="h-8 py-2 bg-background flex justify-end items-center">
		<button
			class="hover:bg-gray-600 px-2 py-1"
			id="titlebar-maximize"
			on:click={() => appWindow.minimize()}
		>
			<MinusIcon class="w-5 h-5 text-white" />
		</button>
		<button
			class="hover:bg-gray-600 px-2 py-1"
			id="titlebar-maximize"
			on:click={async () => {
				await appWindow.toggleMaximize();
				isMaximized = !isMaximized;
			}}
		>
			{#if isMaximized}
				<MinimizeIcon class="w-5 h-5 text-white" />
			{:else}
				<MaximizeIcon class="w-5 h-5 text-white" />
			{/if}
		</button>
		<button
			class="hover:bg-red-600 px-2 py-1"
			id="titlebar-close"
			on:click={() => appWindow.close()}
		>
			<XIcon class="w-7 h-7 text-white" />
		</button>
	</div>

	<div class="w-full h-full bg-background text-secondary px-16 pt-4 pb-8">
		<slot />
	</div>
</div>

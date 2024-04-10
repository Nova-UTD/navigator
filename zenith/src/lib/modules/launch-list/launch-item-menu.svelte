<script lang="ts">
	import * as ContextMenu from '$lib/ui/context-menu';
	import CopyPlusIcon from 'lucide-svelte/icons/copy-plus';
	import FolderPenIcon from 'lucide-svelte/icons/folder-pen';
	import RefreshIcon from 'lucide-svelte/icons/refresh-ccw';
	import LaunchDuplicateDialog from './launch-duplicate-dialog.svelte';
	import { refreshLaunchList } from '$lib/stores/launchStore';
	import { filenameFromPath } from '$lib/utils';
	import LaunchRenameDialog from './launch-rename-dialog.svelte';

	export let name = '';
	export let path = '';
	let filename = '';

	$: filename = filenameFromPath(path);

	let duplicateDialogOpen = false;
	let renameDialogOpen = false;
</script>

<ContextMenu.Root>
	<ContextMenu.Trigger>
		<slot />
	</ContextMenu.Trigger>
	<ContextMenu.Content class="w-64">
		<ContextMenu.Item inset on:click={() => (duplicateDialogOpen = true)}>
			Duplicate
			<ContextMenu.Shortcut><CopyPlusIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
		<ContextMenu.Item inset on:click={() => (renameDialogOpen = true)}>
			Rename
			<ContextMenu.Shortcut><FolderPenIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
		<ContextMenu.Item inset on:click={() => refreshLaunchList()}>
			Reload
			<ContextMenu.Shortcut><RefreshIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
	</ContextMenu.Content>
	<LaunchDuplicateDialog bind:open={duplicateDialogOpen} {name} {filename} />
	<LaunchRenameDialog bind:open={renameDialogOpen} {name} {path} {filename} />
</ContextMenu.Root>

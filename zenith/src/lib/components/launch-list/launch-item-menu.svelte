<script lang="ts">
	import * as ContextMenu from '$lib/components/ui/context-menu';
	import CopyPlusIcon from 'lucide-svelte/icons/copy-plus';
	import FolderPenIcon from 'lucide-svelte/icons/folder-pen';
	import RefreshIcon from 'lucide-svelte/icons/refresh-ccw';
</script>

<ContextMenu.Root>
	<ContextMenu.Trigger>
		<slot />
	</ContextMenu.Trigger>
	<ContextMenu.Content class="w-64">
		<ContextMenu.Item inset>
			Duplicate
			<ContextMenu.Shortcut><CopyPlusIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
		<ContextMenu.Item inset>
			Rename
			<ContextMenu.Shortcut><FolderPenIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
		<ContextMenu.Item inset>
			Reload
			<ContextMenu.Shortcut><RefreshIcon class="w-4 h-4" /></ContextMenu.Shortcut>
		</ContextMenu.Item>
	</ContextMenu.Content>
</ContextMenu.Root>

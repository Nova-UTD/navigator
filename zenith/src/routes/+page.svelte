<script lang="ts">
	import { LaunchEditorPanel } from '$lib/modules/launch-editor';
	import { LaunchTerminalPanel } from '$lib/modules/launch-terminal';
	import { LaunchListPanel } from '$lib/modules/launch-list';
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[0.5fr_1fr_1fr] gap-[5%]">
	<LaunchListPanel />
	<LaunchEditorPanel />
	<LaunchTerminalPanel />
</section>

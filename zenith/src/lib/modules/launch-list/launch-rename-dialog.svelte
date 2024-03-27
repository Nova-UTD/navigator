<script lang="ts">
	import * as Dialog from '$lib/ui/dialog';
	import { Label } from '$lib/ui/label';
	import { Input } from '$lib/ui/input';
	import { Button } from '$lib/ui/button';
	import XIcon from 'lucide-svelte/icons/x';
	import { renameLaunch } from '$lib/stores/launchStore';

	export let open = false;
	export let name = '';
	export let path = '';
	export let filename = '';

	let hasChangedName = false;
	let changedName = name;
	let changedFilename = filename;

	// Automatically change filename when name changes (after first change).
	$: hasChangedName && (changedFilename = `launch.${changedName}.py`);

	let showError = false;
	let emptyError = false;

	$: emptyError = changedName === '' || changedFilename === '';

	function changeName(event: Event) {
		hasChangedName = true;
		showError = false;
		const target = event.target as HTMLInputElement;
		changedName = target.value;
	}

	function changeFilename(event: Event) {
		showError = false;
		const target = event.target as HTMLInputElement;
		changedFilename = target.value;
	}
</script>

<Dialog.Root bind:open>
	<Dialog.Trigger><slot /></Dialog.Trigger>
	<Dialog.Content class="text-white sm:max-w-[425px]">
		<Dialog.Header>
			<Dialog.Title>Rename Launch File</Dialog.Title>
		</Dialog.Header>
		<div class="flex flex-col gap-4 py-4">
			<div class="grid grid-cols-4 items-center gap-4">
				<Label for="name" class="text-right">Name</Label>
				<Input id="name" autofocus on:input={changeName} value={changedName} class="col-span-3" />
			</div>
			<div class="grid grid-cols-4 items-center gap-4">
				<Label for="filename" class="text-right">Filename (excluding path)</Label>
				<Input
					id="filename"
					autofocus
					on:input={changeFilename}
					value={changedFilename}
					class="col-span-3"
				/>
			</div>
			{#if showError && emptyError}
				<div class="flex items-center gap-2 text-xs text-red-500">
					<XIcon class="w-8 h-8" />
					<p>Name and filename cannot be empty.</p>
				</div>
			{/if}
		</div>
		<Dialog.Footer>
			<Button
				on:click={() => {
					if (emptyError) {
						showError = true;
						return;
					}
					renameLaunch({
						newName: changedName,
						newFilename: changedFilename,
						path
					});
					open = false;
				}}
				type="submit"
				variant="secondary">Rename</Button
			>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>

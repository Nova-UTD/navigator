import LaunchEditor from './launch-editor.svelte';
import type { SubsystemInfo } from '$lib/cli-bindings';

export type LaunchEditorProps = {
	subsystems: SubsystemInfo[];
};

export { LaunchEditor };
